# MoveIt2 MCP Server

Model Context Protocol (MCP) server for MoveIt2 motion planning, providing AI assistants with the ability to control and plan robot motions.

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy%20%7C%20Rolling-blue)](https://docs.ros.org/)
[![MoveIt2](https://img.shields.io/badge/MoveIt2-Latest-green)](https://moveit.picknik.ai/)
[![Docker](https://img.shields.io/badge/Docker-Supported-2496ED?logo=docker&logoColor=white)](docs/DOCKER.md)

## Features

- **Full MCP Protocol Support**: Tools, Resources, and Prompts
- **Async Stateful Server**: Non-blocking operations with persistent MoveIt2 state
- **Motion Planning**: Plan to Cartesian poses, joint states, and named configurations
- **Execution Control**: Execute planned trajectories with progress monitoring
- **Planning Scene Management**: Add/remove collision objects dynamically
- **State Queries**: Get current robot state, compute FK/IK, check collisions
- **Resource Providers**: Real-time robot state and planning scene information
- **Multi-Robot Support**: Configuration-driven support for any MoveIt2-compatible robot
- **Multi Transport**: stdio (local), SSE/HTTP (legacy), and Streamable HTTP (MCP 2025-03-26) modes

## Supported Robots

| Robot | Config File | Planning Groups | Description |
|-------|-------------|-----------------|-------------|
| **Franka Emika Panda** | `config/panda_mcp_server.yaml` | `panda_arm`, `panda_hand` | 7-DOF robot arm (default) |
| **Summit XL** | `config/summit_xl_mcp_server.yaml` | `arm`, `gripper` | Mobile manipulator with arm |

Any robot with a MoveIt2 configuration package can be added via a YAML config file.

## Quick Links

- [Quick Start Guide](QUICKSTART.md) - Get up and running in minutes
- [Docker Guide](docs/DOCKER.md) - Run in Docker container
- [Architecture Documentation](docs/ARCHITECTURE.md) - Understand the system design
- [Contributing Guide](CONTRIBUTING.md) - Help improve the project

## Architecture

```
AI Client (Claude) <-> MCP Server <-> moveit_py <-> MoveIt2 Core <-> ROS2
                        |
                        +-- Tools (19)     - Motion planning, execution, scene, queries
                        +-- Resources (4)  - Real-time robot state
                        +-- Prompts (5)    - Guided workflows for common tasks
```

The server maintains a stateful connection to MoveIt2, with async operation tracking for long-running planning and execution tasks. See [ARCHITECTURE.md](docs/ARCHITECTURE.md) for details.

## Prerequisites

- **ROS2**: Rolling (recommended for panda)  or Jazzy
- **MoveIt2**: Installed with Python bindings (`moveit_py`)
- **Robot MoveIt Config**: for your robot e.g., `moveit_resources_panda_moveit_config`
- **Python**: 3.10 or higher

> **Note:** Rolling is fully tested for the Panda arm via Docker. On Jazzy the Panda demo has some known controller spawner issues (see [KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md)).

## Installation

### Option 1: Docker (Recommended)

```bash
git clone <repository-url>
cd moveit-mcp-server

docker-compose build
xhost +local:docker
docker-compose up
```

See [docs/DOCKER.md](docs/DOCKER.md) for complete Docker instructions.

### Option 2: Native Install in ROS environment

```bash
cd moveit-mcp-server
pip install -e ".[dev]"

# Or use setup script
./scripts/setup_environment.sh
```

## Usage

### Panda Robot Arm

#### Terminal 1: Launch MoveIt Demo

```bash
# If using Docker:
xhost +local:docker
docker exec -it moveit-mcp-server bash

# Launch the Panda demo
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

#### Terminal 2: Start MCP Server

```bash
# stdio transport (for local MCP clients)
moveit-mcp-server --config config/panda_mcp_server.yaml

# Streamable HTTP transport (recommended for remote clients / Claude Desktop)
moveit-mcp-server --config config/panda_mcp_server.yaml --transport http --port 8001

# SSE transport (legacy, for older MCP clients)
moveit-mcp-server --config config/panda_mcp_server.yaml --transport sse --port 8000

# Or via Docker
docker exec -it moveit-mcp-server bash -c \
  "moveit-mcp-server-wrapper --config config/panda_mcp_server.yaml --transport http --port 8001"
```

### Summit XL Mobile Manipulator

You can find source code here: https://github.com/icclab/icclab_summit_xl
And a running container image here: ``docker pull robopaas/rosdocked-jazzy-cpu:latest``

Install the moveit-mcp-server, then:

#### Terminal 1: Launch Summit XL MoveIt

```bash
# Launch the Summit XL Sim + MoveIt  (requires icclab_summit_xl_move_it_config)
ros2 launch icclab_summit_xl summit_xl_simulation_ign.launch.py
ros2 launch icclab_summit_xl summit_xl_move_it.launch.py
```

#### Terminal 2: Start MCP Server

```bash
# stdio transport
moveit-mcp-server --config config/summit_xl_mcp_server.yaml

# Streamable HTTP transport
moveit-mcp-server --config config/summit_xl_mcp_server.yaml --transport http --port 8001
```

> **Note:** Summit XL uses `use_sim_time: true` for Gazebo simulation. Ensure your simulation is publishing `/clock`.
> On the real robot, start with `--no-use-sim-time` (there is no `/clock`).
> The warning `config/summit_xl.urdf.xacro doesn't exist ... loaded from /robot_description topic` is expected:
> the robot model comes from `/robot_description`, which on the real robot carries the arm's calibration.

### Connecting Claude Desktop

Add to your Claude Desktop configuration (`~/.config/claude/claude_desktop_config.json`):

```json
{
  "mcpServers": {
    "moveit": {
      "type": "http",
      "url": "http://localhost:8001/mcp"
    }
  }
}
```

> **Legacy SSE clients**: use `"type": "sse"` with `"url": "http://localhost:8000/sse"` and `--transport sse`.

Make sure MoveIt is running before starting Claude Desktop. See [docs/CLAUDE_CODE_SETUP.md](docs/CLAUDE_CODE_SETUP.md) for more details.

## MCP Protocol Support

### Tools (19)

| Category | Tool | Description |
|----------|------|-------------|
| **Planning** | `plan_to_pose` | Plan motion to a Cartesian pose (position + orientation) |
| | `plan_to_joint_state` | Plan motion to target joint positions (radians) |
| | `plan_to_named_state` | Plan motion to a named state (e.g., "ready", "home") |
| | `get_plan_result` | Check planning operation status and result |
| **Execution** | `execute_plan` | Execute a planned trajectory |
| | `get_execution_status` | Monitor execution progress |
| | `stop_execution` | Cancel an ongoing execution |
| | `plan_and_execute` | Combined plan + execute in one call |
| **Scene** | `add_collision_box` | Add a box obstacle to the planning scene |
| | `add_collision_sphere` | Add a sphere obstacle to the planning scene |
| | `remove_collision_object` | Remove a specific collision object |
| | `clear_planning_scene` | Remove all collision objects |
| | `list_collision_objects` | List all collision objects in the scene |
| | `check_state_collision` | Check if a joint state would collide |
| **Queries** | `get_current_joint_state` | Get current joint positions for a group |
| | `get_current_pose` | Get current end-effector pose |
| | `compute_fk` | Compute forward kinematics |
| | `compute_ik` | Compute inverse kinematics |
| | `list_planning_groups` | List available planning groups |

### Resources (4)

| URI | Description |
|-----|-------------|
| `moveit://planning_groups` | Available robot planning groups |
| `moveit://joint_states/{group}` | Current joint positions for a group |
| `moveit://planning_scene` | Collision objects in the scene |
| `moveit://active_operations` | Running async operations |

### Prompts (5)

| Prompt | Description |
|--------|-------------|
| `move_robot_to_pose` | Guided workflow: move robot to a target pose |
| `pick_and_place` | Guided workflow: pick and place an object |
| `inspect_robot_state` | Guided workflow: inspect current robot state |
| `setup_collision_scene` | Guided workflow: add collision objects to scene |
| `plan_cartesian_path` | Guided workflow: plan a multi-waypoint path |

## Example Interactions

### Panda Arm

```
User: "What planning groups are available?"
Claude: Uses list_planning_groups -> "panda_arm", "panda_hand"

User: "Move the panda arm to the ready position"
Claude: Uses plan_to_named_state(group="panda_arm", state_name="ready")
        Then execute_plan -> Robot moves to ready position

User: "What's the current end-effector position?"
Claude: Uses get_current_pose(group="panda_arm")
        -> Position: x=0.307, y=0.000, z=0.590 | Orientation: ...

User: "Add a table at (0.5, 0, 0.4) with size 0.8 x 1.2 x 0.05"
Claude: Uses add_collision_box(object_id="table", position=[0.5,0,0.4],
        dimensions=[0.8,1.2,0.05]) -> Table appears in RViz

User: "Can the arm reach position (0.3, 0.2, 0.5)?"
Claude: Uses compute_ik(group="panda_arm", position=[0.3,0.2,0.5],
        orientation=[0,1,0,0]) -> Returns joint solution or "not reachable"
```

### Summit XL

```
User: "List the planning groups for the Summit XL"
Claude: Uses list_planning_groups -> "arm", "gripper"

User: "Get the arm's current joint state"
Claude: Uses get_current_joint_state(group="arm") -> Joint angles in radians

User: "Plan the arm to its home position"
Claude: Uses plan_to_named_state(group="arm", state_name="home")
        Then execute_plan -> Arm moves to home

User: "Open the gripper"
Claude: Uses plan_to_named_state(group="gripper", state_name="open")
        Then execute_plan -> Gripper opens

User: "Add a box obstacle in front of the robot"
Claude: Uses add_collision_box(object_id="obstacle", position=[0.5,0,0.3],
        dimensions=[0.2,0.2,0.2]) -> Box appears in RViz
```

## Development

```bash
# Install dev dependencies
pip install -e ".[dev]"

# Run tests
pytest

# Run tests with coverage
pytest --cov=moveit_mcp --cov-report=term-missing

# Format code
black src/ tests/

# Lint code
ruff check src/ tests/

# Run unit tests (no ROS required)
pytest tests/ -v -m "not integration"

# Run integration tests (requires ROS/MoveIt)
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py  # Terminal 1
pytest tests/integration/ -v  # Terminal 2

# Run example clients
python examples/panda_example.py
python examples/summit_xl_example.py
```

### Testing

The project includes two test suites:

**Unit Tests** (`tests/`) - No ROS required, uses mocking:
- 71 tests covering all components
- Run with: `pytest tests/ -v -m "not integration"`

**Integration Tests** (`tests/integration/`) - Requires ROS/MoveIt:
- Tests with real ROS2 and MoveIt2
- Verifies actual robot planning and motion
- See [tests/integration/README.md](tests/integration/README.md) for details
- Run with: `pytest tests/integration/ -v`

See [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines.

## License

Apache License 2.0 - See [LICENSE](LICENSE) file for details.

## Contributing

Contributions welcome! Please open an issue or PR on GitHub.
