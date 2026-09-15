import pytest
from blupe_controller.tolerances import near_pose

@pytest.mark.parametrize('gripper', [0.02574002574, 0.0499, [0.02574002574, 0.0499]])
def test_gripper_settles_inside_five_percent(gripper):
    target = [0., 0.] if isinstance(gripper, list) else 0.
    assert near_pose({'joints_deg':[0.]*5, 'gripper':gripper}, [0.]*5, target)

@pytest.mark.parametrize('gripper', [0.05, 0.051, [0., 0.05]])
def test_gripper_at_or_outside_boundary_does_not_settle(gripper):
    target = [0., 0.] if isinstance(gripper, list) else 0.
    assert not near_pose({'joints_deg':[0.]*5, 'gripper':gripper}, [0.]*5, target)

def test_joint_tolerance_remains_five_degrees():
    assert not near_pose({'joints_deg':[5.01]*5, 'gripper':0.}, [0.]*5, 0.)
