"""plan_to_pose: frame_id defaults to the MoveIt planning frame. Runs without ROS."""

import types
from contextlib import contextmanager

import pytest

from moveit_mcp.tools.planning import get_planning_tools


class FakeMoveIt:
    def __init__(self):
        self.calls = []
        scene = types.SimpleNamespace(planning_frame="base_footprint")

        @contextmanager
        def read_only():
            yield scene

        self.planning_scene_monitor = types.SimpleNamespace(read_only=read_only)

    def plan_to_pose(self, **kwargs):
        self.calls.append(kwargs)
        return None


class FakeOpManager:
    async def submit_operation(self, fn, operation_type, metadata=None):
        await fn()
        return "op-1"


ARGS = {"group": "arm", "position": [0.5, 0.0, 0.4], "orientation": [0, 0, 0, 1]}


@pytest.mark.parametrize(
    "extra, expected",
    [({}, "base_footprint"), ({"frame_id": "arm_base_link"}, "arm_base_link")],
)
async def test_frame_id(extra, expected):
    moveit = FakeMoveIt()
    _, handlers = get_planning_tools(moveit, FakeOpManager())
    await handlers["plan_to_pose"]({**ARGS, **extra})
    assert moveit.calls[0]["frame_id"] == expected


def test_schema_names_planning_frame():
    tools, _ = get_planning_tools(FakeMoveIt(), FakeOpManager())
    schema = next(t for t in tools if t.name == "plan_to_pose").inputSchema
    assert "planning frame" in schema["properties"]["frame_id"]["description"]
