"""A running execution leaves the server responsive, and stop_execution stops it. Runs without ROS."""

import asyncio
import re
import sys
import threading
import time
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

from moveit_mcp.async_ops import AsyncOperationManager, OperationType  # noqa: E402
import moveit_mcp.moveit_wrapper as wrapper  # noqa: E402
from moveit_mcp.tools.execution import get_execution_tools  # noqa: E402


class FakeMoveIt:
    """execute_trajectory blocks like MoveItPy.execute() until stopped, at most 2 s."""

    def __init__(self):
        self.stopped = threading.Event()

    def execute_trajectory(self, trajectory, controllers=None):
        return not self.stopped.wait(2.0)

    def stop_execution(self):
        self.stopped.set()


async def _start_execution(moveit, ops):
    async def plan():
        return "trajectory"

    plan_id = await ops.submit_operation(plan, OperationType.PLANNING)
    await ops.wait_for_operation(plan_id)
    _, handlers = get_execution_tools(moveit, ops)
    text = (await handlers["execute_plan"]({"operation_id": plan_id}))[0].text
    exec_id = re.search(r"Operation ID: (\S+)", text).group(1)
    await asyncio.sleep(0.1)  # execution is running
    return handlers, exec_id


async def test_status_answers_while_the_arm_moves():
    moveit, ops = FakeMoveIt(), AsyncOperationManager()
    handlers, exec_id = await _start_execution(moveit, ops)
    start = time.monotonic()
    text = (await handlers["get_execution_status"]({"operation_id": exec_id}))[0].text
    assert time.monotonic() - start < 0.5 and "running" in text
    moveit.stop_execution()


async def test_stop_execution_stops_the_motion():
    moveit, ops = FakeMoveIt(), AsyncOperationManager()
    handlers, exec_id = await _start_execution(moveit, ops)
    await handlers["stop_execution"]({"operation_id": exec_id})
    assert moveit.stopped.is_set()


def test_wrapper_stops_through_the_trajectory_execution_manager():
    calls = []
    tem = types.SimpleNamespace(stop_execution=lambda auto_clear: calls.append(auto_clear))
    moveit = types.SimpleNamespace(get_trajectory_execution_manager=lambda: tem)
    wrapper.MoveItWrapper.stop_execution(types.SimpleNamespace(moveit=moveit))
    assert calls == [True]
