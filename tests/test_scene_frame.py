"""add_collision_box/sphere: frame_id defaults to the planning frame, unknown frames are refused. Runs without ROS."""

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
from moveit_mcp.tools.scene import get_scene_tools  # noqa: E402


class FakeScene:
    planning_frame = "base_footprint"

    def knows_frame_transform(self, frame_id):
        return frame_id in ("base_footprint", "arm_base_link")


@pytest.mark.parametrize("frame_id, expected", [(None, "base_footprint"), ("", "base_footprint"),
                                                ("arm_base_link", "arm_base_link")])
def test_scene_frame(frame_id, expected):
    assert MoveItWrapper._scene_frame(None, FakeScene(), frame_id) == expected


def test_unknown_frame_refused():
    with pytest.raises(ValueError, match="Unknown frame 'world'"):
        MoveItWrapper._scene_frame(None, FakeScene(), "world")


class FakeMoveIt:
    def __init__(self):
        self.calls = []

    def add_collision_box(self, object_id, pose, dimensions, frame_id):
        self.calls.append(frame_id)

    def add_collision_sphere(self, object_id, pose, radius, frame_id):
        self.calls.append(frame_id)


@pytest.mark.parametrize("tool, args", [
    ("add_collision_box", {"object_id": "b", "position": [1, 0, 0.5], "dimensions": [0.1, 0.1, 0.1]}),
    ("add_collision_sphere", {"object_id": "s", "position": [1, 0, 0.5], "radius": 0.1}),
])
async def test_handler_passes_no_default_frame(tool, args):
    moveit = FakeMoveIt()
    _, handlers = get_scene_tools(moveit)
    await handlers[tool](args)
    await handlers[tool]({**args, "frame_id": "arm_base_link"})
    assert moveit.calls == [None, "arm_base_link"]


def test_schema_names_planning_frame():
    tools, _ = get_scene_tools(FakeMoveIt())
    for t in tools:
        if t.name in ("add_collision_box", "add_collision_sphere"):
            assert "planning frame" in t.inputSchema["properties"]["frame_id"]["description"]
