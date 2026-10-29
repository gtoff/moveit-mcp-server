"""planning.sensors_3d: false clears the depth sensors (octomap); without the key they stay. Runs without ROS."""

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

SENSORS = {"sensors": ["arm_camera"]}


def _apply(config):
    moveit_config = types.SimpleNamespace(sensors_3d=dict(SENSORS))
    wrapper.MoveItWrapper._apply_sensors(types.SimpleNamespace(config=config), moveit_config)
    return moveit_config.sensors_3d


def test_sensors_off():
    assert _apply({"planning": {"sensors_3d": False}}) == {}


def test_sensors_stay_by_default():
    assert _apply({"planning": {}}) == SENSORS
