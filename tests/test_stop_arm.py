"""stop_arm stops and latches, resume_arm releases. Runs without ROS."""

import asyncio
import threading

import pytest

from tests.test_stop_execution import FakeMoveIt, _start_execution
from moveit_mcp.async_ops import AsyncOperationManager, OperationType
from moveit_mcp.tools import worker
from moveit_mcp.tools.execution import get_execution_tools
from moveit_mcp.tools.planning import get_planning_tools


@pytest.fixture(autouse=True)
def released():
    worker.stop_latched.clear()
    yield
    worker.stop_latched.clear()


async def _plan_id(ops):
    async def plan():
        return "trajectory"

    plan_id = await ops.submit_operation(plan, OperationType.PLANNING)
    await ops.wait_for_operation(plan_id)
    return plan_id


async def test_stop_arm_stops_the_motion_and_fails_it():
    moveit, ops = FakeMoveIt(), AsyncOperationManager()
    handlers, exec_id = await _start_execution(moveit, ops)
    assert (await handlers["stop_arm"]({}))[0].text == "Arm stop confirmed"
    assert moveit.stopped.is_set() and not worker.executing.is_set()
    status = await ops.get_operation_status(exec_id)
    assert status["status"] == "failed" and status["error"] == worker.STOPPED


async def test_latched_refuses_motions_and_plans_until_resume():
    moveit, ops = FakeMoveIt(), AsyncOperationManager()
    _, handlers = get_execution_tools(moveit, ops)
    _, planning = get_planning_tools(moveit, ops)
    await handlers["stop_arm"]({})
    plan_id = await _plan_id(ops)
    assert (await handlers["execute_plan"]({"operation_id": plan_id}))[0].text == worker.LATCHED_TEXT
    text = (await planning["plan_to_joint_state"]({"group": "arm", "joint_positions": [0.0]}))[0].text
    assert text == worker.LATCHED_TEXT
    assert (await handlers["resume_arm"]({}))[0].text == "Arm resumed"
    assert "Execution started" in (await handlers["execute_plan"]({"operation_id": plan_id}))[0].text
    moveit.stop_execution()


async def test_execute_refuses_to_start_once_latched():
    started = threading.Event()

    def fake_execute():
        started.set()

    worker.latch()
    with pytest.raises(RuntimeError, match=worker.STOPPED):
        await worker.on_execute_thread(fake_execute)
    assert not started.is_set()
