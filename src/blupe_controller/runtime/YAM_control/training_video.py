"""Post-run camera montage; no camera or motor ownership, only saved frames."""
import bisect
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ORDER = ('left', 'top', 'right')
# Bump when rendering changes; preserve the deployed cache contract.
RENDER_VERSION = 4


def motion_frames(rows, count):
    """Keep movement and 0.4s margins; remove still intervals of at least 1.5s.

    Unknown feedback gaps are retained, never classified as stopped motion.
    """
    times = [r['timestamp'] for r in rows]
    active = []
    for n in range(count):
        stamp = n / 10
        window = rows[bisect.bisect_left(times, stamp - .2):bisect.bisect_right(times, stamp + .3)]
        if len(window) < 2 or any(b['timestamp'] - a['timestamp'] > .2 for a, b in zip(window, window[1:])):
            active.append(True)
            continue
        q = [r['measured_joints'] for r in window]
        g = [r['measured_grippers'] for r in window]
        active.append(any(max(v)-min(v) >= .008 for v in zip(*q)) or
                      any(max(v)-min(v) >= .025 for v in zip(*g)))
    padded = [any(active[max(0, n-4):n+5]) for n in range(count)]
    keep = [True] * count
    start = 0
    while start < count:
        if padded[start]:
            start += 1
            continue
        end = start + 1
        while end < count and not padded[end]:
            end += 1
        if end-start >= 15:
            keep[start:end] = [False] * (end-start)
        start = end
    return [i for i, flag in enumerate(keep) if flag] or [0]


def render_video(episode, output, *, preview=False):
    from PIL import Image, ImageDraw
    episode, output = Path(episode), Path(output)
    meta = json.loads((episode / 'manifest.json').read_text())
    if meta['status'] != 'finalized':
        raise ValueError('Video requires a finalized recording')
    rows = [json.loads(line) for line in (episode / 'samples.jsonl').read_text().splitlines()]
    times = [r['timestamp'] for r in rows]
    if not times or any(not math.isfinite(t) or t < 0 for t in times) or times != sorted(times):
        raise ValueError('Invalid video timeline')
    duration = times[-1] + .1
    if meta.get('ended_monotonic') is not None:
        duration = max(duration, meta['ended_monotonic'] - meta['started_monotonic'])
    if not 0 < duration <= 3600:
        raise ValueError('Invalid video duration')
    encoder = os.environ.get('YAM_FFMPEG') or shutil.which('ffmpeg')
    if not encoder:
        raise RuntimeError('FFmpeg is required for run video export')
    output.parent.mkdir(parents=True, exist_ok=True)
    partial = output.with_suffix('.partial.mp4')
    order = tuple(meta['camera_devices']) if meta.get('robot_id', 'yam-1') != 'yam-1' else (('top', 'observer', 'left', 'right') if preview and any('observer_image_path' in r for r in rows) else ORDER)
    if not 1 <= len(order) <= 4:
        raise ValueError('Video requires one to four named cameras')
    width, height = (1280, 768) if len(order) == 4 else (640 * len(order), 384)
    # See docs/refs/ffmpeg: explicit RGB24 geometry/rate, H.264 yuv420p +faststart.
    command = [encoder, '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pixel_format', 'rgb24',
               '-video_size', f'{width}x{height}', '-framerate', '30' if preview else '10', '-i', '-', '-an',
               '-c:v', 'libx264', '-threads', '1', '-preset', 'veryfast', '-crf', '23',
               '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(partial)]
    count = math.ceil(duration * 10 - 1e-6)  # tolerate monotonic subtraction round-off
    selected = motion_frames(rows, count) if preview else range(count)
    # Select output frames BEFORE JPEG decode and montage composition. At 10x,
    # only 30/100 input ticks appear in the 30 FPS output.
    encoded = [selected[min(len(selected)-1, int(i*100/30))]
               for i in range(max(1, round(len(selected)*30/100)))] if preview else selected
    # Camera samples can be absent independently of joint samples. Hold only
    # past images, for at most three seconds; never pull a future frame forward.
    camera_rows = {name: [r for r in rows if r.get(name + '_image_path')] for name in order}
    camera_times = {name: [r['timestamp'] for r in camera_rows[name]] for name in order}
    # End at the last recorded sample with usable imagery in every available
    # camera. A late stop/cleanup timestamp is not additional video footage.
    available = [name for name in order if camera_rows[name]]
    final_stamp = None
    for candidate in reversed(times):
        if available and all(
            (idx := bisect.bisect_right(camera_times[name], candidate) - 1) >= 0
            and candidate - camera_times[name][idx]
                + max(0, camera_rows[name][idx].get(name + '_frame_age_s', 0)) <= 3.0
            for name in available
        ):
            final_stamp = candidate
            break
    if final_stamp is None:
        raise ValueError('No recorded camera frame available for video')
    encoded = [n for n in encoded if n / 10 < final_stamp]
    encoded.append(final_stamp * 10)  # Include the actual last view even at 10x.
    # Viewing previews omit unavailable footage instead of flashing black tiles.
    # Full-speed exports retain gaps so their original timing stays explicit.
    before_gap_cut = len(encoded)
    if preview:
        encoded = [n for n in encoded if all(
            (idx := bisect.bisect_right(camera_times[name], n / 10) - 1) >= 0
            and n / 10 - camera_times[name][idx]
                + max(0, camera_rows[name][idx].get(name + '_frame_age_s', 0)) <= 3.0
            for name in available
        )]
    missing = 0
    held = 0
    missing_cameras = 0
    with tempfile.TemporaryFile() as errors:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=errors)
        try:
            for n in encoded:
                stamp = n / 10
                index = bisect.bisect_right(times, stamp) - 1
                row = rows[index] if index >= 0 and stamp - times[index] <= .15 else None
                canvas = Image.new('RGB', (width, height), '#15191e')
                draw = ImageDraw.Draw(canvas)
                if row is None:
                    missing += 1
                for col, name in enumerate(order):
                    x, y = ((col % 2) * 640, (col // 2) * 384) if len(order) == 4 else (col * 640, 0)
                    label = f'{"SIDE" if name == "observer" else name.upper()}  |  {stamp:.1f}s' + ('  |  10x' if preview else '')
                    camera_index = bisect.bisect_right(camera_times[name], stamp) - 1
                    camera_row = camera_rows[name][camera_index] if camera_index >= 0 else None
                    age = (stamp - camera_row['timestamp'] + max(0, camera_row.get(name + '_frame_age_s', 0))) if camera_row else float('inf')
                    if camera_row is not None and age <= 3.0:
                        path = (episode / camera_row[name + '_image_path']).resolve()
                        if path.parent != (episode / 'images').resolve():
                            raise ValueError('Image path escapes recording')
                        with Image.open(path) as source:
                            tile = source.convert('RGB')
                            tile.thumbnail((640, 360))
                            canvas.paste(tile, (x + (640-tile.width)//2, y + 24 + (360-tile.height)//2))
                        if age > .15:
                            held += 1
                        if age > .5:
                            label += '  |  LAST RECORDED FRAME'
                    else:
                        missing_cameras += 1
                        draw.text((x+16, y+175), 'NO RECORDED FRAME', fill='white')
                    draw.text((x+10, y+6), label, fill='white')
                process.stdin.write(canvas.tobytes())
            process.stdin.close()
            if process.wait(timeout=120) != 0:
                raise RuntimeError('Video encoder failed')
            partial.replace(output)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            if process.stdin and not process.stdin.closed:
                process.stdin.close()
            partial.unlink(missing_ok=True)
    return {'camera_order': list(order), 'fps': 30 if preview else 10, 'frames': len(encoded),
            'duration_s': len(encoded)/(30 if preview else 10), 'missing_frame_ticks': missing, 'held_camera_ticks': held, 'missing_camera_ticks': missing_cameras, 'render_version': RENDER_VERSION, 'unavailable_ticks_removed': before_gap_cut-len(encoded), 'trimmed_tail_s': max(0, duration-final_stamp-.1),
            **({'speed': 10, 'idle_removed_s': (count-len(selected))/10} if preview else {})}
