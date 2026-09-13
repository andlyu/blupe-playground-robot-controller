import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from blupe_controller.cli import main, validate


class SetupTests(unittest.TestCase):
    def config(self):
        return {'version':1, 'hardware':'yam','robot_id':'test-yam','api':'https://api.example',
                'token_file':'/tmp/device-credential', 'cameras':{'left':1,'top':2,'right':3}}

    def test_setup_is_private_and_does_not_start_hardware(self):
        with tempfile.TemporaryDirectory() as directory, patch('subprocess.Popen', side_effect=AssertionError('Process started')):
            path=Path(directory)/'config.json'
            args=['--config',str(path),'setup','--robot-id','test-yam','--api','https://api.example','--token-file',str(Path(directory)/'credential'),'--cameras','1','2','3']
            with contextlib.redirect_stdout(io.StringIO()): self.assertEqual(main(args),0)
            self.assertEqual(path.stat().st_mode & 0o777,0o600)
            saved=json.loads(path.read_text());self.assertEqual(saved['settings'],{})
            self.assertEqual(saved['cameras'],{'left':1,'top':2,'right':3})
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): main(args)

    def test_reject_unsupported_hardware(self):
        with self.assertRaises(ValueError): validate({**self.config(),'hardware':'so101'})

    def test_invalid_origins_and_identity(self):
        for value in ['http://api.example','https://secret@api.example','https://api.example/path','https://api.example/?token=x']:
            with self.assertRaises(ValueError): validate({**self.config(),'api':value})
        with self.assertRaises(ValueError): validate({**self.config(),'robot_id':'../wrong'})

    def test_duplicate_cameras_rejected(self):
        with self.assertRaises(ValueError): validate({**self.config(),'cameras':{'left':1,'top':1,'right':2}})

    def test_doctor_does_not_open_devices(self):
        from blupe_controller.cli import doctor
        with patch('subprocess.Popen', side_effect=AssertionError('Process started')), contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(doctor(self.config()))


if __name__=='__main__': unittest.main()

class TunnelTests(unittest.TestCase):
    def test_only_assigned_loopback_port_and_strict_host_verification(self):
        with tempfile.TemporaryDirectory() as directory:
            key=Path(directory)/'key'; key.write_text('fixture'); key.chmod(0o600)
            hosts=Path(directory)/'hosts'; hosts.write_text('fixture')
            with patch('blupe_controller.cli.subprocess.call',return_value=0) as call:
                self.assertEqual(main(['connect-operator','--user','robot-isaac','--remote-port','28096','--identity',str(key),'--known-hosts',str(hosts)]),0)
                command=call.call_args.args[0]
                self.assertIn('127.0.0.1:28096:127.0.0.1:8096',command)
                self.assertIn('StrictHostKeyChecking=yes',command)
                self.assertIn('ExitOnForwardFailure=yes',command)
                self.assertNotIn('0.0.0.0',str(command))
            key.chmod(0o644)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit), patch('subprocess.call',side_effect=AssertionError('Started')):
                main(['connect-operator','--user','robot-isaac','--remote-port','28096','--identity',str(key),'--known-hosts',str(hosts)])

class BackendTests(unittest.TestCase):
    def test_dispatch_is_lazy_and_uses_selected_backend(self):
        from blupe_controller.backends import run
        for name in ('yam','so101'):
            with patch('blupe_controller.backends.import_module') as load:
                run({'hardware':name})
                load.assert_called_once_with('blupe_controller.backends.'+name)
                load.return_value.run.assert_called_once_with({'hardware':name})
        with self.assertRaises(ValueError): run({'hardware':'unknown'})

    def test_shared_template_matches_yam_and_so101_binding(self):
        import ast
        from blupe_controller import operator_page
        page=operator_page.PAGE
        self.assertIn('Run via Session API',page)
        self.assertIn('s.at_home!==true',page)
        self.assertNotIn('function near(s,p)',page)
        self.assertNotIn('__import__',page)
