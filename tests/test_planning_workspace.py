"""The configured workspace reaches each new PlanningComponent. Runs without ROS."""

import sys
import types

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

import moveit_mcp.moveit_wrapper as wrapper  # noqa: E402

BOX = [-2.0, -2.0, -1.0, 2.0, 2.0, 2.0]


class FakeComponent:
    def __init__(self):
        self.workspace = None

    def set_workspace(self, *bounds):
        self.workspace = list(bounds)


def _wrapper(config):
    model = types.SimpleNamespace(has_joint_model_group=lambda name: True)
    moveit = types.SimpleNamespace(get_planning_component=lambda name: FakeComponent())
    return types.SimpleNamespace(config=config, robot_model=model, moveit=moveit, planning_components={})


def test_workspace_from_config():
    w = _wrapper({"planning": {"workspace": BOX}})
    assert wrapper.MoveItWrapper.get_planning_component(w, "arm").workspace == BOX


def test_without_workspace_none_is_set():
    w = _wrapper({"planning": {}})
    assert wrapper.MoveItWrapper.get_planning_component(w, "arm").workspace is None
