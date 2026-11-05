"""compute_fk: link transforms are computed from the given joints. Runs without ROS."""

import sys
import types
from contextlib import contextmanager

# Minimal geometry_msgs stand-in, only where ROS is missing
try:
    import geometry_msgs.msg  # noqa: F401
except ImportError:
    class _Xyz:
        x = y = z = 0.0

    class _Quat(_Xyz):
        w = 1.0

    class Pose:
        def __init__(self):
            self.position = _Xyz()
            self.orientation = _Quat()

    msg = types.ModuleType("geometry_msgs.msg")
    msg.Pose = Pose
    sys.modules["geometry_msgs"] = types.ModuleType("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = msg

from moveit_mcp import moveit_wrapper  # noqa: E402
from moveit_mcp.moveit_wrapper import MoveItWrapper  # noqa: E402


class FakeRobotState:
    """Like moveit_py's RobotState: link transforms stay zero until update()."""

    def __init__(self, robot_model):
        self.joint_positions = {}
        self.group_positions = None
        self.updated = False

    def set_joint_group_positions(self, group, positions):
        self.group_positions = list(positions)
        self.updated = False

    def update(self):
        self.updated = True

    def get_global_link_transform(self, link_name):
        return ("transform", self.group_positions, dict(self.joint_positions)) if self.updated else "zeros"


def make_wrapper():
    wrapper = MoveItWrapper.__new__(MoveItWrapper)
    group = types.SimpleNamespace(link_model_names=["base", "tool0"])
    wrapper.robot_model = types.SimpleNamespace(get_joint_model_group=lambda name: group)
    scene = types.SimpleNamespace(
        current_state=types.SimpleNamespace(joint_positions={"finger_joint": 0.3})
    )

    @contextmanager
    def read_only():
        yield scene

    wrapper.planning_scene_monitor = types.SimpleNamespace(read_only=read_only)
    return wrapper


def test_fk_uses_updated_transforms(monkeypatch):
    monkeypatch.setattr(moveit_wrapper, "RobotState", FakeRobotState, raising=False)
    monkeypatch.setattr(moveit_wrapper, "transform_matrix_to_pose", lambda transform: transform)

    result = make_wrapper().compute_fk("arm", [0.1, 0.2])

    assert result == ("transform", [0.1, 0.2], {"finger_joint": 0.3})
