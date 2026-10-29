"""A per-call planner_id and planning_time reach the planner through PlanRequestParameters. Runs without ROS."""

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


class FakeParams:
    def __init__(self, moveit, namespace):
        self.namespace = namespace
        self.planner_id = "from config"


class FakeComponent:
    """PlanningComponent as in MoveItPy: plan() only, no set_planner_id()."""

    def __init__(self):
        self.calls = []

    def plan(self, single_plan_parameters=None):
        self.calls.append(single_plan_parameters)
        return "plan"


def test_planner_id_goes_through_request_parameters(monkeypatch):
    monkeypatch.setattr(wrapper, "PlanRequestParameters", FakeParams, raising=False)
    component = FakeComponent()
    assert wrapper.MoveItWrapper._plan(types.SimpleNamespace(moveit=None), component, "RRTConnect") == "plan"
    (params,) = component.calls
    assert params.planner_id == "RRTConnect" and params.namespace == ""


def test_without_planner_id_the_config_applies():
    component = FakeComponent()
    wrapper.MoveItWrapper._plan(types.SimpleNamespace(moveit=None), component, None)
    assert component.calls == [None]


def test_planning_time_goes_through_request_parameters(monkeypatch):
    monkeypatch.setattr(wrapper, "PlanRequestParameters", FakeParams, raising=False)
    component = FakeComponent()
    wrapper.MoveItWrapper._plan(types.SimpleNamespace(moveit=None), component, None, 1.5)
    (params,) = component.calls
    assert params.planning_time == 1.5 and params.planner_id == "from config"
