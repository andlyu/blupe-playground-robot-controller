import json
import os
from pathlib import Path
import pty
import select
import subprocess
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from blupe_controller.so101 import NAMES, SO101Driver, calibration


class ServoEmulator:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        self.writes = []
        self.reads = 0
        self.position = [2000] * 6
        self.bad_offset = False
        self.bad_checksum = False
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        pending = b''
        while not self.done.is_set():
            if not select.select([self.master], [], [], .05)[0]:
                continue
            try:
                pending += os.read(self.master, 4096)
            except OSError:
                continue
            while len(pending) >= 4 and len(pending) >= pending[3] + 4:
                length = pending[3] + 4
                packet, pending = pending[:length], pending[length:]
                if packet[:2] != b'\xff\xff' or sum(packet[2:]) % 256 != 255:
                    raise AssertionError('Invalid native packet')
                ident, instruction = packet[2], packet[4]
                if instruction == 2:
                    self.reads += 1
                    address, size = packet[5:7]
                    value = {3:777, 31:1 if self.bad_offset else 0, 33:0, 56:self.position[ident-1]}[address]
                    reply = [ident, size+2, 0, value & 255]
                    if size == 2:
                        reply.append(value >> 8)
                    checksum = (~sum(reply)) & 255
                    os.write(self.master, bytes([255,255,*reply,checksum ^ int(self.bad_checksum)]))
                else:
                    self.writes.append((time.monotonic(), packet))
                    if packet[5] == 42:
                        for start in range(7, len(packet)-1, 3):
                            servo, lo, hi = packet[start:start+3]
                            self.position[servo-1] = lo | hi << 8

    def close(self):
        self.done.set()
        self.thread.join(1)
        os.close(self.master)
        os.close(self.slave)


class NativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.builddir = tempfile.TemporaryDirectory()
        cls.binary = Path(cls.builddir.name) / 'driver'
        source = Path(__file__).parents[1] / 'src/blupe_controller/native/so101.cpp'
        subprocess.run(['clang++', '-std=c++17', '-DBLUPE_TEST_PTY', '-O2', '-Wall', '-Wextra', '-pthread', str(source), '-o', str(cls.binary)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.builddir.cleanup()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / 'calibration.json'
        self.path.write_text(json.dumps({name:dict(id=i+1, drive_mode=0, homing_offset=0, range_min=1000, range_max=3000) for i,name in enumerate(NAMES)}))
        self.bus = ServoEmulator()
        self.config = dict(settings=dict(serial_port=self.bus.port, calibration_file=str(self.path)), cameras={'front':0})
        self.build_patch = patch('blupe_controller.so101.build', return_value=self.binary)
        self.build_patch.start()
        self.driver = SO101Driver(self.config)

    def tearDown(self):
        self.driver.close()
        self.build_patch.stop()
        self.bus.close()
        self.temp.cleanup()

    def test_readonly_no_writes_and_joint_units(self):
        self.driver.connect()
        time.sleep(.15)
        self.assertEqual(self.bus.writes, [])
        self.assertGreater(self.bus.reads, 30)
        state = self.driver.state()
        self.assertEqual(state['joints_deg'], [0]*5)
        self.assertEqual(state['gripper'], .5)

    def test_native_ramp_and_hold(self):
        self.driver.connect()
        self.driver.enable()
        self.driver.move([10]*5, .6)
        time.sleep(.2)
        current = self.driver.state()['raw'][0]
        self.assertGreater(current, 2000)
        self.assertLess(current, 2040)
        self.driver.hold()
        time.sleep(.08)
        held = self.bus.position[:]
        time.sleep(.12)
        self.assertEqual(self.bus.position, held)

    def test_watchdog_runs_without_python_heartbeats(self):
        self.driver.connect()
        self.driver.enable()
        self.driver.move([20]*5, .6)
        self.driver.done.set()  # Stop IPC heartbeat; native loop keeps running.
        time.sleep(.7)
        self.assertEqual(self.driver.state()['mode'], 'hold')
        self.assertEqual(self.driver.state()['error'], 'heartbeat_timeout')
        held = self.bus.position[:]
        reads = self.bus.reads
        time.sleep(.15)
        self.assertEqual(self.bus.position, held)
        self.assertGreater(self.bus.reads, reads)

    def test_native_keeps_polling_when_python_process_is_suspended(self):
        import signal
        import sys
        program = """
import time
from pathlib import Path
import blupe_controller.so101 as module
module.build = lambda: Path(BINARY)
driver = module.SO101Driver(CONFIG).connect()
try:
    driver.enable()
    driver.move([20]*5, .6)
    print('ready', flush=True)
    time.sleep(5)
finally:
    driver.close()
""".replace('BINARY', repr(str(self.binary))).replace('CONFIG', repr(self.config))
        parent = subprocess.Popen([sys.executable, '-c', program], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            self.assertTrue(select.select([parent.stdout], [], [], 3)[0])
            self.assertEqual(parent.stdout.readline().strip(), 'ready')
            os.kill(parent.pid, signal.SIGSTOP)
            before = self.bus.reads
            time.sleep(.7)
            self.assertGreater(self.bus.reads, before + 30)
            held = self.bus.position[:]
            self.assertLess(held[0], 2080)
            time.sleep(.15)
            self.assertEqual(self.bus.position, held)
        finally:
            os.kill(parent.pid, signal.SIGCONT)
            parent.terminate()
            parent.wait(timeout=3)
            parent.stdout.close()
            parent.stderr.close()
            time.sleep(.1)  # Native worker observes parent-pipe EOF and exits.

    def test_native_rejects_bypassed_limits(self):
        self.driver.connect()
        self.driver.enable()
        with self.assertRaisesRegex(ValueError, 'target_outside_limits'):
            self.driver._send('target 5000 2000 2000 2000 2000 2000')
        self.assertEqual(self.bus.position, [2000]*6)
        with self.assertRaises(ValueError):
            self.driver.enable()

    def test_calibration_mismatch_never_writes(self):
        self.bus.bad_offset = True
        with self.assertRaisesRegex(ValueError, 'calibration_mismatch'):
            self.driver.connect()
        self.assertEqual(self.bus.writes, [])

    def test_bad_feedback_latches_fault(self):
        self.driver.connect()
        self.bus.bad_checksum = True
        time.sleep(.1)
        self.assertEqual(self.driver.state()['mode'], 'fault')
        self.assertEqual(self.bus.writes, [])

    def test_invalid_targets_rejected_before_ipc(self):
        for angles, gripper in [([0]*4,.5),([float('nan')]*5,.5),([0]*5,2),([1000]*5,.5)]:
            with self.assertRaises(ValueError):
                self.driver.move(angles, gripper)

    def test_duplicate_calibration_ids_rejected(self):
        data = json.loads(self.path.read_text())
        data['gripper']['id'] = 1
        self.path.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            calibration(self.path)

class OperatorTests(unittest.TestCase):
    def test_home_is_captured_not_assumed(self):
        from unittest.mock import Mock
        from blupe_controller.operator import Operator
        driver = Mock()
        driver.state.return_value = {'mode':'readonly', 'joints_deg':[1,2,3,4,5], 'gripper':.4}
        operator = Operator(driver, {'robot_id':'test','cameras':{'front':0}})
        with self.assertRaises(ValueError):
            operator.action({'action':'home'})
        operator.action({'action':'capture_home'})
        driver.enable.assert_not_called()
        driver.move.assert_not_called()
        operator.action({'action':'home'})
        driver.move.assert_called_once_with([1,2,3,4,5], .4)

    def test_so101_setup_private_and_camera_names(self):
        from blupe_controller.cli import main, load
        with tempfile.TemporaryDirectory() as directory:
            cal=Path(directory)/'cal.json'
            cal.write_text(json.dumps({name:dict(id=i+1,drive_mode=0,homing_offset=0,range_min=1000,range_max=3000) for i,name in enumerate(NAMES)}))
            output=Path(directory)/'config.json'
            with patch('subprocess.Popen',side_effect=AssertionError('Hardware opened')):
                main(['--config',str(output),'setup','--hardware','so101','--robot-id','test','--api','https://example.com','--token-file',str(Path(directory)/'credential'),'--serial-port','/dev/example','--calibration',str(cal),'--camera','front=0'])
            self.assertEqual(load(output)['cameras'],{'front':0})
            self.assertEqual(output.stat().st_mode&0o777,0o600)

    def test_panel_rejects_missing_token_and_cross_origin(self):
        import io
        from unittest.mock import Mock
        from blupe_controller.operator import serve
        driver = Mock()
        config = {'robot_id':'test','cameras':{'front':0},'settings':{}}
        captured = {}
        def server_factory(address, handler):
            captured['handler'] = handler
            return Mock()
        with patch('blupe_controller.operator.ThreadingHTTPServer', side_effect=server_factory), patch('blupe_controller.operator.secrets.token_urlsafe',return_value='test-secret'):
            serve(driver,config)
        def request(headers):
            handler = object.__new__(captured['handler'])
            handler.headers = headers
            handler.path = '/api/action'
            handler.respond = Mock()
            handler.connection = Mock()
            handler.rfile = io.BytesIO(b'{"action":"enable"}')
            handler.do_POST()
            return handler.respond.call_args.args[0]
        self.assertEqual(request({'Host':'localhost:8096'}),403)
        self.assertEqual(request({'Host':'evil.example','X-Blupe-Control':'test-secret'}),403)
        self.assertEqual(request({'Host':'localhost:8096','X-Blupe-Control':'test-secret','Origin':'https://evil.example'}),403)
        driver.enable.assert_not_called()
        self.assertEqual(request({'Host':'localhost:8096','X-Blupe-Control':'test-secret','Origin':'http://localhost:8096','Content-Length':'19'}),200)
        driver.enable.assert_called_once()
