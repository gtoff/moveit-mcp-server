"""execute_plan needs the plan's operation_id: there is no 'last plan' to fall back on. Runs without ROS."""

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

from moveit_mcp.async_ops import AsyncOperationManager  # noqa: E402
from moveit_mcp.tools.execution import get_execution_tools  # noqa: E402


async def test_execute_without_id_is_refused():
    _, handlers = get_execution_tools(object(), AsyncOperationManager())
    text = (await handlers["execute_plan"]({}))[0].text
    assert text.startswith("Error:") and "operation_id" in text
