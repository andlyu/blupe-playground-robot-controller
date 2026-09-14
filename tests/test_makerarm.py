import pytest
from blupe_controller.makerarm import MakerArmDriver, validate_profile
from blupe_controller.poses import PoseStore
from blupe_controller.backends import BACKENDS

def config():
    return dict(hardware='makerarm',robot_id='test',cameras={'front':0}, settings=dict(channel='can0',gripper_endpoints_rad=[-.1,-2.]))

def test_validation_without_hardware():
    assert validate_profile(config())
    for change in ({'max_velocity':float('nan')},{'gripper_endpoints_rad':[0,0]},{'channel':''}):
        c=config();c['settings'].update(change)
        with pytest.raises(ValueError):validate_profile(c)
    assert BACKENDS['makerarm']=='makerarm'

def test_pose_six_joints():
    store=PoseStore(config())
    pose=store.capture('home',dict(mode='readonly',joints_deg=[1]*6,gripper=.5))
    assert len(pose['joints_deg'])==6
    with pytest.raises(ValueError):store.capture('zero',dict(mode='readonly',joints_deg=[1]*5,gripper=.5))

def test_target_validation_and_gripper_contract():
    driver=MakerArmDriver(config());driver.limits=[[-90,90]]*6
    driver._validate_target([0]*6,.5)
    assert driver._action([0]*6,.5)['gripper.pos']==.5
    for joints,grip in [([0]*5,.5),([100]*6,.5),([0]*6,2)]:
        with pytest.raises(ValueError):driver._validate_target(joints,grip)

def test_worker_connect_is_readonly_and_targets_require_enable(monkeypatch, tmp_path):
    import sys
    import types
    from enum import Enum
    from unittest.mock import Mock
    import importlib.resources
    from blupe_controller.makerarm import _worker
    class State(Enum):
        CONNECTED=1
        ENABLED=2
        FAULT=3
    arm=Mock()
    arm.state=State.CONNECTED
    arm.config=types.SimpleNamespace(joints=[types.SimpleNamespace(lo=-3,hi=3) for _ in range(7)],feedback_timeout=.2)
    arm.motors=[types.SimpleNamespace(feedback_age=0) for _ in range(7)]
    arm.refresh.return_value=[True]*7
    arm.get_joint_positions.return_value=[0]*6+[-1]
    fake=types.ModuleType('maker_arm.arm');fake.Arm=Mock();fake.Arm.from_yaml.return_value=arm;fake.ArmState=State
    monkeypatch.setitem(sys.modules,'maker_arm.arm',fake)
    monkeypatch.setattr(importlib.resources,'files',lambda _:tmp_path)
    class Pipe:
        def __init__(self):self.messages=[];self.commands=iter([('state',None),('move',([0]*6,.5)),('close',None)])
        def send(self,value):self.messages.append(value)
        def poll(self,timeout):return True
        def recv(self):return next(self.commands)
    pipe=Pipe();_worker(pipe,config()['settings'])
    arm.enable.assert_not_called()
    arm.set_joint_targets.assert_not_called()
    assert pipe.messages[1]['mode']=='readonly'
    assert 'enable' in pipe.messages[2]['error']
    arm.disconnect.assert_called()
