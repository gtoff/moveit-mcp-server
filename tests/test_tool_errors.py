"""Tool refusals are logged as one warning line, real errors with their traceback. Runs without ROS."""

import logging
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

from moveit_mcp.tools.errors import log_tool_error  # noqa: E402
from moveit_mcp.tools.scene import get_scene_tools  # noqa: E402


def test_refusal_is_a_warning_without_traceback(caplog):
    with caplog.at_level(logging.WARNING):
        log_tool_error(logging.getLogger("t"), "add_collision_box", ValueError("Unknown frame 'nowhere'"))
    (record,) = caplog.records
    assert record.levelno == logging.WARNING and record.exc_info is None
    assert record.getMessage() == "add_collision_box refused: Unknown frame 'nowhere'"


def test_other_error_keeps_the_traceback(caplog):
    try:
        raise RuntimeError("MoveIt died")
    except RuntimeError as e:
        with caplog.at_level(logging.WARNING):
            log_tool_error(logging.getLogger("t"), "execute_plan", e)
    (record,) = caplog.records
    assert record.levelno == logging.ERROR and record.exc_info is not None


async def test_handler_refusal_logs_one_line(caplog):
    _, handlers = get_scene_tools(object())
    with caplog.at_level(logging.WARNING):
        text = (await handlers["add_collision_box"]({"object_id": "b", "position": [1, 0], "dimensions": [0.1] * 3}))[0].text
    assert text.startswith("Error:")
    (record,) = [r for r in caplog.records if r.name.startswith("moveit_mcp")]
    assert record.levelno == logging.WARNING and record.exc_info is None
