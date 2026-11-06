"""get_current_pose / compute_fk: the output names the frame of the pose. Runs without ROS."""

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

from geometry_msgs.msg import Pose  # noqa: E402

from moveit_mcp.tools.queries import get_query_tools  # noqa: E402


class FakeMoveIt:
    robot_model = types.SimpleNamespace(model_frame="base_footprint")

    def get_current_pose(self, group, link_name):
        return Pose()

    def compute_fk(self, group, joint_positions, link_name):
        return Pose()


@pytest.mark.parametrize("tool, args", [
    ("get_current_pose", {"group": "arm"}),
    ("compute_fk", {"group": "arm", "joint_positions": [0.0] * 6}),
])
async def test_output_names_frame(tool, args):
    _, handlers = get_query_tools(FakeMoveIt())
    result = await handlers[tool](args)
    assert "in frame 'base_footprint'" in result[0].text
