"""--use-sim-time and --sensors-3d (and their --no- forms) override the config file. Runs without ROS."""

from moveit_mcp.server import apply_overrides


def test_no_flag_keeps_config():
    config = {"mcp_server": {"use_sim_time": True}}
    apply_overrides(config, use_sim_time=None)
    assert config["mcp_server"]["use_sim_time"] is True


def test_flag_overrides_config():
    config = {"mcp_server": {"use_sim_time": True}}
    apply_overrides(config, use_sim_time=False)
    assert config["mcp_server"]["use_sim_time"] is False


def test_config_without_mcp_server_section():
    config = {}
    apply_overrides(config, use_sim_time=True)
    assert config["mcp_server"]["use_sim_time"] is True


def test_no_sensors_3d_flag_overrides_config():
    config = {"planning": {"sensors_3d": True}}
    apply_overrides(config, sensors_3d=False)
    assert config["planning"]["sensors_3d"] is False
