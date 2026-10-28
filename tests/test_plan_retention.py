"""A removed or unknown plan ID gets a clear answer naming the keep time. Runs without ROS."""

import sys
import types

import pytest

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

from moveit_mcp.async_ops import AsyncOperationManager, OperationType  # noqa: E402
from moveit_mcp.tools.execution import get_execution_tools  # noqa: E402
from moveit_mcp.tools.planning import get_planning_tools  # noqa: E402


def test_not_found_text_names_the_timeout():
    text = AsyncOperationManager(operation_timeout=600.0).not_found_text("op-9")
    assert "op-9 not found" in text and "600 s" in text


async def test_execute_removed_plan_says_so():
    manager = AsyncOperationManager(operation_timeout=600.0)
    _, handlers = get_execution_tools(object(), manager)
    text = (await handlers["execute_plan"]({"operation_id": "gone"}))[0].text
    assert "gone not found" in text and "Plan again" in text


async def test_get_plan_result_removed_plan_says_so():
    manager = AsyncOperationManager(operation_timeout=600.0)
    _, handlers = get_planning_tools(object(), manager)
    text = (await handlers["get_plan_result"]({"operation_id": "gone"}))[0].text
    assert "gone not found" in text and "600 s" in text


async def test_finished_plans_do_not_block_new_ones():
    manager = AsyncOperationManager(max_concurrent_operations=2, operation_timeout=600.0)

    async def plan():
        return "trajectory"

    for _ in range(3):  # more finished plans than the limit
        operation_id = await manager.submit_operation(plan, OperationType.PLANNING)
        await manager.operations[operation_id][0]
    assert len(manager.operations) == 3
