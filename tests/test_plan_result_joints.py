"""get_plan_result shows the plan's joint goal and each joint's move. Runs without ROS."""

import math
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

from moveit_mcp.moveit_wrapper import MoveItWrapper  # noqa: E402
from moveit_mcp.tools.planning import get_planning_tools  # noqa: E402


def fake_plan(names, points):
    """A plan result whose trajectory message has these joint names and points."""
    trajectory = types.SimpleNamespace(
        joint_names=names, points=[types.SimpleNamespace(positions=p) for p in points])
    msg = types.SimpleNamespace(joint_trajectory=trajectory)
    return types.SimpleNamespace(
        trajectory=types.SimpleNamespace(get_robot_trajectory_msg=lambda: msg))


def test_trajectory_end():
    plan = fake_plan(["pan", "wrist_3"], [[0.0, 0.0], [0.1, 0.0], [0.5, -math.pi]])
    end = MoveItWrapper.trajectory_end(None, plan)
    assert end["joint_names"] == ["pan", "wrist_3"]
    assert end["joint_positions"] == [0.5, -math.pi]
    assert end["deltas"] == pytest.approx([0.5, -math.pi])


@pytest.mark.parametrize("plan", [object(), fake_plan(["pan"], [])])
def test_trajectory_end_without_points(plan):
    assert MoveItWrapper.trajectory_end(None, plan) is None


class FakeMoveIt:
    trajectory_end = MoveItWrapper.trajectory_end


class FakeOpManager:
    def __init__(self, result):
        self.result = result

    async def get_operation_status(self, operation_id):
        return {"status": "completed", "progress": 1.0, "result": self.result}


async def plan_text(result):
    _, handlers = get_planning_tools(FakeMoveIt(), FakeOpManager(result))
    return (await handlers["get_plan_result"]({"operation_id": "op-1"}))[0].text


async def test_get_plan_result_shows_joint_goal():
    text = await plan_text(fake_plan(["arm_wrist_3_joint"], [[0.0], [-3.1244]]))
    assert "Planning succeeded" in text
    assert "arm_wrist_3_joint: -3.1244 rad (-179.0 deg)" in text


async def test_get_plan_result_without_trajectory():
    text = await plan_text(object())
    assert "Planning succeeded" in text
    assert "Joint goal" not in text
