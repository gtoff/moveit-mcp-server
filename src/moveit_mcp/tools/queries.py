"""Query tool implementations for robot state information."""

import logging
from typing import Any, Dict, Optional

from ..moveit_wrapper import MoveItWrapper
from .errors import log_tool_error

try:
    from geometry_msgs.msg import Pose
    from mcp.server import Server
    from mcp.types import Tool, TextContent
except ImportError:
    pass

logger = logging.getLogger(__name__)


def get_query_tools(moveit: MoveItWrapper, server_instance=None) -> tuple[list, dict]:
    """
    Get query tool definitions and handlers.

    Returns:
        Tuple of (tools, handlers) where:
        - tools: List of Tool definitions
        - handlers: Dict mapping tool name to handler function
    """

    def check_initialized():
        """Check if server is initialized before processing requests."""
        if server_instance and not server_instance.is_initialized():
            raise RuntimeError("Server not fully initialized. Please wait for initialization to complete.")

    # Define individual tool handler functions
    async def get_current_joint_state_handler(arguments: dict) -> list:
        """
        Get current joint positions for a planning group.

        Args (from arguments dict):
            group: Planning group name

        Returns:
            Current joint names and positions
        """
        try:
            check_initialized()
            group = arguments["group"]
            state = moveit.get_current_state(group)

            result_text = f"Current joint state for '{group}':\n"
            for name, pos in zip(state["joint_names"], state["joint_positions"]):
                result_text += f"  {name}: {pos:.4f} rad\n"

            return [TextContent(type="text", text=result_text)]

        except Exception as e:
            log_tool_error(logger, "get_current_joint_state", e)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def get_current_pose_handler(arguments: dict) -> list:
        """
        Get current end-effector pose.

        Args (from arguments dict):
            group: Planning group name
            link_name: Optional link name (uses end-effector if not specified)

        Returns:
            Current pose
        """
        try:
            check_initialized()
            group = arguments["group"]
            link_name = arguments.get("link_name")

            pose = moveit.get_current_pose(group, link_name)

            result_text = f"Current pose for '{group}':\n"
            result_text += f"Position:\n"
            result_text += f"  x: {pose.position.x:.4f} m\n"
            result_text += f"  y: {pose.position.y:.4f} m\n"
            result_text += f"  z: {pose.position.z:.4f} m\n"
            result_text += f"Orientation (quaternion):\n"
            result_text += f"  x: {pose.orientation.x:.4f}\n"
            result_text += f"  y: {pose.orientation.y:.4f}\n"
            result_text += f"  z: {pose.orientation.z:.4f}\n"
            result_text += f"  w: {pose.orientation.w:.4f}\n"

            return [TextContent(type="text", text=result_text)]

        except Exception as e:
            log_tool_error(logger, "get_current_pose", e)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def compute_fk_handler(arguments: dict) -> list:
        """
        Compute forward kinematics for given joint positions.

        Args (from arguments dict):
            group: Planning group name
            joint_positions: Joint positions in radians
            link_name: Optional link name

        Returns:
            Computed pose
        """
        try:
            check_initialized()
            group = arguments["group"]
            joint_positions = arguments["joint_positions"]
            link_name = arguments.get("link_name")

            pose = moveit.compute_fk(group, joint_positions, link_name)

            result_text = f"Forward kinematics result for '{group}':\n"
            result_text += f"Position:\n"
            result_text += f"  x: {pose.position.x:.4f} m\n"
            result_text += f"  y: {pose.position.y:.4f} m\n"
            result_text += f"  z: {pose.position.z:.4f} m\n"
            result_text += f"Orientation (quaternion):\n"
            result_text += f"  x: {pose.orientation.x:.4f}\n"
            result_text += f"  y: {pose.orientation.y:.4f}\n"
            result_text += f"  z: {pose.orientation.z:.4f}\n"
            result_text += f"  w: {pose.orientation.w:.4f}\n"

            return [TextContent(type="text", text=result_text)]

        except Exception as e:
            log_tool_error(logger, "compute_fk", e)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def compute_ik_handler(arguments: dict) -> list:
        """
        Compute inverse kinematics for a target pose.

        Args (from arguments dict):
            group: Planning group name
            position: Target position [x, y, z] in meters
            orientation: Target orientation [x, y, z, w] quaternion
            timeout: IK timeout in seconds (default: 5.0)
            attempts: Number of IK attempts (default: 10)

        Returns:
            Joint positions if IK succeeded, error otherwise
        """
        try:
            check_initialized()
            group = arguments["group"]
            position = arguments["position"]
            orientation = arguments["orientation"]
            timeout = arguments.get("timeout", 5.0)
            attempts = arguments.get("attempts", 10)

            # Validate inputs
            if len(position) != 3:
                raise ValueError("Position must have 3 elements [x, y, z]")
            if len(orientation) != 4:
                raise ValueError("Orientation must have 4 elements [x, y, z, w]")

            # Create pose (ensure all values are floats for ROS)
            pose = Pose()
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])
            pose.orientation.x = float(orientation[0])
            pose.orientation.y = float(orientation[1])
            pose.orientation.z = float(orientation[2])
            pose.orientation.w = float(orientation[3])

            # Compute IK
            joint_positions = moveit.compute_ik(group, pose, timeout, attempts)

            if joint_positions is not None:
                result_text = f"Inverse kinematics solution for '{group}':\n"
                state = moveit.get_current_state(group)
                for name, pos in zip(state["joint_names"], joint_positions):
                    result_text += f"  {name}: {pos:.4f} rad\n"
                return [TextContent(type="text", text=result_text)]
            else:
                return [
                    TextContent(
                        type="text",
                        text=f"IK failed: No solution found for target pose after {attempts} attempts.",
                    )
                ]

        except Exception as e:
            log_tool_error(logger, "compute_ik", e)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def list_planning_groups_handler(arguments: dict) -> list:
        """
        List all available planning groups.

        Returns:
            List of planning group names
        """
        try:
            check_initialized()
            groups = moveit.get_planning_groups()

            result_text = f"Available planning groups ({len(groups)}):\n"
            for group in groups:
                result_text += f"  - {group}\n"

            return [TextContent(type="text", text=result_text)]

        except Exception as e:
            log_tool_error(logger, "list_planning_groups", e)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    # Define tool specifications
    tools = [
        Tool(
            name="get_current_joint_state",
            description="Get current joint positions for a planning group",
            inputSchema={
                "type": "object",
                "properties": {
                    "group": {"type": "string", "description": "Planning group name"}
                },
                "required": ["group"],
            },
        ),
        Tool(
            name="get_current_pose",
            description="Get current end-effector pose",
            inputSchema={
                "type": "object",
                "properties": {
                    "group": {"type": "string", "description": "Planning group name"},
                    "link_name": {
                        "type": "string",
                        "description": "Optional link name (uses end-effector if not specified)",
                    },
                },
                "required": ["group"],
            },
        ),
        Tool(
            name="compute_fk",
            description="Compute forward kinematics for given joint positions",
            inputSchema={
                "type": "object",
                "properties": {
                    "group": {"type": "string", "description": "Planning group name"},
                    "joint_positions": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Joint positions in radians",
                    },
                    "link_name": {"type": "string", "description": "Optional link name"},
                },
                "required": ["group", "joint_positions"],
            },
        ),
        Tool(
            name="compute_ik",
            description="Compute inverse kinematics for a target pose",
            inputSchema={
                "type": "object",
                "properties": {
                    "group": {"type": "string", "description": "Planning group name"},
                    "position": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Target position [x, y, z] in meters",
                    },
                    "orientation": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Target orientation quaternion [x, y, z, w]",
                    },
                    "timeout": {
                        "type": "number",
                        "description": "IK timeout in seconds (default: 5.0)",
                    },
                    "attempts": {
                        "type": "integer",
                        "description": "Number of IK attempts (default: 10)",
                    },
                },
                "required": ["group", "position", "orientation"],
            },
        ),
        Tool(
            name="list_planning_groups",
            description="List all available planning groups",
            inputSchema={"type": "object", "properties": {}},
        ),
    ]

    # Map handlers
    handlers = {
        "get_current_joint_state": get_current_joint_state_handler,
        "get_current_pose": get_current_pose_handler,
        "compute_fk": compute_fk_handler,
        "compute_ik": compute_ik_handler,
        "list_planning_groups": list_planning_groups_handler,
    }

    return tools, handlers


def register_query_tools(server: "Server", moveit: MoveItWrapper, server_instance=None):
    """
    Register query/information MCP tools (legacy compatibility).

    Note: This function is deprecated. Use get_query_tools() with
    the unified ToolRegistry instead.
    """
    from .registry import ToolRegistry

    tools, handlers = get_query_tools(moveit, server_instance)
    registry = ToolRegistry()
    registry.register_tools(tools, handlers)
    registry.setup_server_handlers(server)
