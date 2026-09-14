import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch
import pytest
from blupe_controller.bimanual_so101 import BimanualSO101Monitor, validate_profile


def config():
    return dict(version=1,hardware='bimanual_so101',robot_id='local-so101-pair',cameras={'overhead':6},settings={'arms':{'arm_a':'/dev/test-a','arm_b':'/dev/test-b'}})


def test_read_only_buses():
    ports=[]
    def port(device):
        result=Mock();ports.append(result);return result
    packet=Mock();packet.ping.return_value=(777,0,0);packet.read2ByteTxRx.return_value=(2048,0,0);packet.read1ByteTxRx.return_value=(0,0,0)
    with patch.dict(sys.modules,scservo_sdk=SimpleNamespace(PortHandler=port,PacketHandler=lambda _:packet)), patch('blupe_controller.bimanual_so101.Path.exists',return_value=False):
        driver=BimanualSO101Monitor(config()).connect()
        state=driver.state()
        assert len(state['arms'])==2 and all(len(a['motors'])==6 for a in state['arms'])
        assert not state['calibration_ready']
        with pytest.raises(ValueError):driver.enable()
        with pytest.raises(ValueError):driver.move([],0)
        assert {c[0] for c in packet.method_calls}=={'ping','read2ByteTxRx','read1ByteTxRx'}
        with patch('blupe_controller.bimanual_so101.Path.exists',return_value=True):
            assert driver.state()['arms']==[]
            for p in ports:p.closePort.assert_called_once()


def test_no_cloud_or_duplicate_bus():
    c=config();c['settings']['cloud_enabled']=True
    with pytest.raises(ValueError):validate_profile(c)
    c=config();c['settings']['arms']['arm_b']='/dev/test-a'
    with pytest.raises(ValueError):validate_profile(c)


def test_bimanual_rejects_second_arm_before_writing():
    from blupe_controller.bimanual_so101 import BimanualSO101Driver
    import threading
    class Arm:
        def __init__(self): self.moves=[]
        def _validate_target(self,j,g):
            if any(abs(v)>90 for v in j): raise ValueError('limit')
        def state(self): return dict(mode='active',error='',joints_deg=[0]*5,gripper=.5)
        def move(self,j,g): self.moves.append(j)
    driver=BimanualSO101Driver.__new__(BimanualSO101Driver)
    driver.drivers=[Arm(),Arm()]; driver.lock=threading.RLock()
    import pytest
    with pytest.raises(ValueError): driver.move([0]*5+[180]*5,[.5,.5])
    assert not any(d.moves for d in driver.drivers)


def test_bimanual_cloud_preserves_right_arm():
    from blupe_controller.cloud import CloudBridge
    from blupe_controller.tolerances import near_pose
    bridge=CloudBridge.__new__(CloudBridge)
    bridge.config={'hardware':'bimanual_so101','api':'https://example.com','robot_id':'test','cameras':{}}
    class Driver:
        def state(self): return dict(joints_deg=[0]*10,gripper=[.2,.7])
        def _validate_target(self,j,g): assert len(j)==10 and len(g)==2
    bridge.driver=Driver()
    joints=list(range(10))
    targets=bridge.targets([dict(left_joints_deg=joints[:5],right_joints_deg=joints[5:],right_gripper=.9)])
    assert targets==[(joints,[.2,.9])]
    fields=bridge.image_fields(dict(joints_deg=joints,gripper=[.2,.9]))
    assert fields['right_joints_deg']==joints[5:] and fields['right_gripper']==.9
    assert not near_pose(dict(joints_deg=joints,gripper=[.2,.9]),joints,[.2,.5])
