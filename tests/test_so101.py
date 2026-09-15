import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from blupe_controller.so101 import NAMES, SO101Driver


class LeRobotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)/'cal.json'
        self.path.write_text(json.dumps({name:dict(id=i+1,drive_mode=0,homing_offset=0,range_min=1000,range_max=3000) for i,name in enumerate(NAMES)}))
        self.config = {'settings':{'serial_port':'/dev/example','calibration_file':str(self.path)}, 'cameras':{'front':0}}
        self.robot = Mock()
        self.robot.is_calibrated = True
        self.robot.bus.is_connected = True
        self.robot.bus.read.return_value = 0
        self.robot.get_observation.return_value = {**{name+'.pos':0 for name in NAMES[:5]},'gripper.pos':50}
        self.robot.send_action.side_effect = lambda action: action
        self.factory = patch('blupe_controller.so101.make_robot',return_value=self.robot)
        self.factory.start()
        self.driver = SO101Driver(self.config)

    def tearDown(self):
        self.driver.close()
        self.factory.stop()
        self.temp.cleanup()

    def test_probe_uses_lerobot_without_writes(self):
        self.driver.connect()
        self.assertEqual(self.driver.state()['gripper'],.5)
        self.robot.bus.sync_write.assert_not_called()
        self.robot.bus.enable_torque.assert_not_called()
        self.robot.connect.assert_not_called()
        self.driver.close()
        self.robot.bus.disconnect.assert_called_once_with(disable_torque=False)

    def test_move_requires_enable_and_maps_gripper(self):
        self.driver.connect()
        with self.assertRaises(ValueError): self.driver.move([1]*5,.6)
        self.driver.enable()
        self.robot.bus.enable_torque.assert_called_once()
        result=self.driver.move([1,2,3,4,5],.6)
        self.assertEqual(result['sent_action']['gripper.pos'],60)
        self.assertEqual(result['sent_action']['wrist_roll.pos'],5)

    def test_disable_verifies_every_motor_and_requires_reenable(self):
        self.driver.connect()
        self.driver.enable()
        self.driver.disable()
        self.robot.bus.disable_torque.assert_called_once()
        for name in NAMES:
            self.robot.bus.read.assert_any_call('Torque_Enable', name, normalize=False)
        self.assertEqual(self.driver.mode, 'readonly')
        with self.assertRaises(ValueError): self.driver.move([0]*5, .5)
        self.driver.enable()
        self.assertEqual(self.driver.mode, 'active')

    def test_unverified_disable_latches_fault(self):
        self.driver.connect()
        self.robot.bus.read.return_value = 1
        with self.assertRaisesRegex(ValueError, 'torque-off verification failed'):
            self.driver.disable()
        self.assertEqual(self.driver.mode, 'fault')

    def test_limits_and_nonfinite_targets_rejected(self):
        self.driver.connect()
        self.driver.enable()
        for angles,gripper in [([1000]*5,.5),([0]*4,.5),([float('nan')]*5,.5),([0]*5,2)]:
            with self.assertRaises(ValueError): self.driver.move(angles,gripper)
        self.robot.send_action.assert_not_called()

    def test_hold_uses_measured_pose(self):
        self.driver.connect()
        self.driver.enable()
        self.driver.move([10]*5,.6)
        self.driver.hold()
        self.assertEqual(self.robot.bus.sync_write.call_args.args[1]['gripper'],50)
        with self.assertRaises(ValueError): self.driver.move([0]*5,.5)

    def test_calibration_mismatch_never_enables(self):
        self.robot.is_calibrated=False
        with self.assertRaises(ValueError): self.driver.connect()
        self.robot.bus.enable_torque.assert_not_called()

    def test_disconnect_respects_robot_config_after_enable(self):
        self.driver.config['settings']['disable_torque_on_disconnect']=True
        self.driver.connect()
        self.driver.enable()
        self.driver.close()
        self.robot.bus.disconnect.assert_called_once_with(disable_torque=True)

    def test_config_reference_is_reread_and_not_duplicated(self):
        from blupe_controller.cli import main, load
        source=Path(self.temp.name)/'robot.json'
        source.write_text(json.dumps({'type':'so101_follower','port':'/dev/first','id':'cal',
            'calibration_dir':self.temp.name,'use_degrees':True,'max_relative_target':5,
            'cameras':{'front':{'type':'opencv','index_or_path':0,'width':640,'height':480,'fps':30}}}))
        output=Path(self.temp.name)/'controller.json'
        main(['--config',str(output),'setup','--hardware','so101','--robot-id','cloud-id',
            '--api','https://example.com','--token-file',str(Path(self.temp.name)/'credential'),
            '--lerobot-config',str(source)])
        saved=json.loads(output.read_text())
        self.assertNotIn('cameras',saved)
        self.assertNotIn('serial_port',saved['settings'])
        self.assertEqual(load(output)['settings']['max_relative_target'],5)
        data=json.loads(source.read_text());data['port']='/dev/second';source.write_text(json.dumps(data))
        self.assertEqual(load(output)['settings']['serial_port'],'/dev/second')


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
        with self.assertRaises(ValueError):
            operator.action({'action':'home'})
        driver.move.assert_not_called()

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


class RelativeLimitTests(unittest.TestCase):
    def test_integer_json_limit_is_converted_for_lerobot(self):
        from blupe_controller.so101 import relative_limit
        self.assertIs(type(relative_limit(5)),float)
        for value in (True,0,-1,float('nan')):
            with self.assertRaises(ValueError):relative_limit(value)
