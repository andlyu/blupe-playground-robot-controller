#!/usr/bin/env python3
"""Publish fresh relay snapshots to the Session API; never opens robot devices."""
import argparse
import concurrent.futures
import math
from pathlib import Path
import time
import urllib.request
import urllib.error
from urllib.parse import quote

MAX_BYTES = 4 * 1024 * 1024


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def capture_time(headers, now_wall, now_mono):
    captured = float(headers['X-Capture-Monotonic'])
    age = now_mono - captured
    if not math.isfinite(age) or not 0 <= age <= 1.5:
        raise ValueError('No fresh capture timestamp')
    return now_wall - age


def publish(args, role, camera):
    last_sequence = None
    failing = False
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    while True:
        started = time.monotonic()
        try:
            with opener.open(f'{args.camera_origin}/{camera}/snapshot.jpg', timeout=2) as response:
                captured = capture_time(response.headers, time.time(), time.monotonic())
                sequence = response.headers['X-Frame-Sequence']
                jpeg = response.read(MAX_BYTES + 1)
            if sequence is None or len(jpeg) > MAX_BYTES or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
                raise ValueError('Invalid camera snapshot')
            if sequence != last_sequence:
                token = Path(args.token_file).read_text().strip()
                request = urllib.request.Request(
                    f'{args.api.rstrip("/")}/v1/robots/{quote(args.jetson_id, safe="")}/cameras/{role}.jpg',
                    data=jpeg, method='PUT', headers={'Authorization': 'Bearer ' + token,
                    'Content-Type': 'image/jpeg', 'X-Captured-At': str(captured)})
                with opener.open(request, timeout=3) as response:
                    if response.status != 204:
                        raise ValueError('Unexpected upload response')
                last_sequence = sequence
                if failing:
                    print(f'[camera-api] {role} recovered', flush=True)
                failing = False
        except Exception as error:
            if not failing:
                detail = f'HTTP {error.code}' if isinstance(error, urllib.error.HTTPError) else type(error).__name__
                print(f'[camera-api] {role} unavailable ({detail})', flush=True)
            failing = True
        time.sleep(max(0.01, (1.0 if failing else 0.2) - (time.monotonic() - started)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--api', required=True)
    parser.add_argument('--jetson-id', required=True)
    parser.add_argument('--token-file', required=True)
    parser.add_argument('--camera-origin', default='http://127.0.0.1:8089')
    parser.add_argument('--observer-only', action='store_true', help='Publish device 18 as the viewing-only observer camera')
    args = parser.parse_args()
    if not args.api.startswith('https://'):
        parser.error('API uploads require HTTPS')
    cameras = [('observer', 18)] if args.observer_only else [('left', 10), ('top', 16), ('right', 4)]
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(cameras)) as pool:
        futures = [pool.submit(publish, args, role, camera) for role, camera in cameras]
        for future in futures:
            future.result()


if __name__ == '__main__':
    main()
