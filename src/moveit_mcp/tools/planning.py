"""Planning tool implementations."""

import logging
from typing import Any, Dict, Optional

from ..async_ops import AsyncOperationManager, OperationType
from ..moveit_wrapper import MoveItWrapper

try:
    from geometry_msgs.msg import Pose
    from mcp.server import Server
    from mcp.types import Tool, TextContent
except ImportError:
    pass

logger = logging.getLogger(__name__)


def get_planning_tools(
    moveit: MoveItWrapper, op_manager: AsyncOperationManager, server_instance=None
) -> tuple[list, dict]:
    """
    Get planning tool definitions and handlers.

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
    async def plan_to_pose_handler(arguments: dict) -> list:
        """
        Plan robot motion to a target Cartesian pose.

        Args (from arguments dict):
            group: Planning group name (e.g., "panda_arm")
            position: Target position [x, y, z] in meters
            orientation: Target orientation as quaternion [x, y, z, w]
            frame_id: Reference frame (default: the MoveIt planning frame)
            planner_id: Optional planner ID (default: use configured)
            timeout: Planning timeout in seconds (default: 5.0)

        Returns:
            Operation ID for async tracking
        """
        try:
            check_initialized()
            group = arguments["group"]
            position = arguments["position"]
            orientation = arguments["orientation"]
            # Caller's frame, else the MoveIt planning frame
            if "frame_id" in arguments:
                frame_id = arguments["frame_id"]
            else:
                with moveit.planning_scene_monitor.read_only() as scene:
                    frame_id = scene.planning_frame
            planner_id = arguments.get("planner_id")
            timeout = arguments.get("timeout", 5.0)

            # Validate inputs
            if len(position) != 3:
                raise ValueError("Position must have 3 elements [x, y, z]")
            if len(orientation) != 4:
                raise ValueError("Orientation must have 4 elements [x, y, z, w]")

            # Create Pose message (ensure all values are floats for ROS)
            target_pose = Pose()
            target_pose.position.x = float(position[0])
            target_pose.position.y = float(position[1])
            target_pose.position.z = float(position[2])
            target_pose.orientation.x = float(orientation[0])
            target_pose.orientation.y = float(orientation[1])
            target_pose.orientation.z = float(orientation[2])
            target_pose.orientation.w = float(orientation[3])

            # Submit async operation
            async def plan_fn():
                return moveit.plan_to_pose(
                    group_name=group,
                    target_pose=target_pose,
                    frame_id=frame_id,
                    planner_id=planner_id,
                    planning_time=timeout,
                )

            operation_id = await op_manager.submit_operation(
                plan_fn,
                OperationType.PLANNING,
                metadata={"group": group, "target": "pose"},
            )

            return [
                TextContent(
                    type="text",
                    text=f"Planning operation submitted. Operation ID: {operation_id}\n"
                    f"Use get_plan_result with this ID to check status.",
                )
            ]

        except Exception as e:
            logger.error(f"Error in plan_to_pose: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def plan_to_joint_state_handler(arguments: dict) -> list:
        """
        Plan robot motion to target joint positions.

        Args (from arguments dict):
            group: Planning group name
            joint_positions: List of target joint positions in radians
            planner_id: Optional planner ID
            timeout: Planning timeout in seconds (default: 5.0)

        Returns:
            Operation ID for async tracking
        """
        try:
            check_initialized()
            group = arguments["group"]
            joint_positions = arguments["joint_positions"]
            planner_id = arguments.get("planner_id")
            timeout = arguments.get("timeout", 5.0)

            # Submit async operation
            async def plan_fn():
                return moveit.plan_to_joint_state(
                    group_name=group,
                    joint_positions=joint_positions,
                    planner_id=planner_id,
                    planning_time=timeout,
                )

            operation_id = await op_manager.submit_operation(
                plan_fn,
                OperationType.PLANNING,
                metadata={"group": group, "target": "joint_state"},
            )

            return [
                TextContent(
                    type="text",
                    text=f"Planning operation submitted. Operation ID: {operation_id}\n"
                    f"Use get_plan_result with this ID to check status.",
                )
            ]

        except Exception as e:
            logger.error(f"Error in plan_to_joint_state: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def plan_to_named_state_handler(arguments: dict) -> list:
        """
        Plan robot motion to a named state (e.g., "ready", "home").

        Args (from arguments dict):
            group: Planning group name
            state_name: Name of the target state
            planner_id: Optional planner ID

        Returns:
            Operation ID for async tracking
        """
        try:
            check_initialized()
            group = arguments["group"]
            state_name = arguments["state_name"]
            planner_id = arguments.get("planner_id")

            # Submit async operation
            async def plan_fn():
                return moveit.plan_to_named_state(
                    group_name=group, state_name=state_name, planner_id=planner_id
                )

            operation_id = await op_manager.submit_operation(
                plan_fn,
                OperationType.PLANNING,
                metadata={"group": group, "target": "named_state", "state_name": state_name},
            )

            return [
                TextContent(
                    type="text",
                    text=f"Planning operation submitted. Operation ID: {operation_id}\n"
                    f"Use get_plan_result with this ID to check status.",
                )
            ]

        except Exception as e:
            logger.error(f"Error in plan_to_named_state: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def get_plan_result_handler(arguments: dict) -> list:
        """
        Get the result of a planning operation.

        Args (from arguments dict):
            operation_id: Operation ID returned from planning call

        Returns:
            Operation status and result
        """
        try:
            check_initialized()
            operation_id = arguments["operation_id"]
            status = await op_manager.get_operation_status(operation_id)

            if status is None:
                return [TextContent(type="text", text=f"Operation {operation_id} not found")]

            result_text = f"Operation: {operation_id}\n"
            result_text += f"Status: {status['status']}\n"
            result_text += f"Progress: {status['progress'] * 100:.1f}%\n"

            if status["status"] == "completed":
                if status["result"]:
                    result_text += "Planning succeeded! Trajectory is ready for execution.\n"
                else:
                    result_text += "Planning failed: No valid path found.\n"
            elif status["status"] == "failed":
                result_text += f"Error: {status['error']}\n"
            elif status["status"] == "running":
                result_text += "Planning is still in progress...\n"

            return [TextContent(type="text", text=result_text)]

        except Exception as e:
            logger.error(f"Error in get_plan_result: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    # Define tool specifications
    tools = [
        Tool(
            name="plan_to_pose",
            description="Plan robot motion to a target Cartesian pose",
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
                    "frame_id": {
                        "type": "string",
                        "description": "Reference frame (default: the MoveIt planning frame)",
                    },
                    "planner_id": {"type": "string", "description": "Optional planner ID"},
                    "timeout": {
                        "type": "number",
                        "description": "Planning timeout in seconds",
                    },
                },
                "required": ["group", "position", "orientation"],
            },
        ),
        Tool(
            name="plan_to_joint_state",
            description="Plan robot motion to target joint positions",
            inputSchema={
                "type": "object",
                "properties": {
                    "group": {"type": "string", "description": "Planning group name"},
                    "joint_positions": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Target joint positions in radians",
                    },
                    "planner_id": {"type": "string", "description": "Optional planner ID"},
                    "timeout": {
                        "type": "number",
                        "description": "Planning timeout in seconds",
                    },
                },
                "required": ["group", "joint_positions"],
            },
        ),
        Tool(
            name="plan_to_named_state",
            description="Plan robot motion to a named state (e.g., 'ready', 'home')",
            inputSchema={
                "type": "object",
                "properties": {
                    "group": {"type": "string", "description": "Planning group name"},
                    "state_name": {"type": "string", "description": "Name of target state"},
                    "planner_id": {"type": "string", "description": "Optional planner ID"},
                },
                "required": ["group", "state_name"],
            },
        ),
        Tool(
            name="get_plan_result",
            description="Get the result/status of a planning operation",
            inputSchema={
                "type": "object",
                "properties": {
                    "operation_id": {
                        "type": "string",
                        "description": "Operation ID from planning call",
                    }
                },
                "required": ["operation_id"],
            },
        ),
    ]

    # Map handlers
    handlers = {
        "plan_to_pose": plan_to_pose_handler,
        "plan_to_joint_state": plan_to_joint_state_handler,
        "plan_to_named_state": plan_to_named_state_handler,
        "get_plan_result": get_plan_result_handler,
    }

    return tools, handlers


def register_planning_tools(
    server: "Server", moveit: MoveItWrapper, op_manager: AsyncOperationManager
):
    """
    Register planning-related MCP tools (legacy compatibility).

    Note: This function is deprecated. Use get_planning_tools() with
    the unified ToolRegistry instead.
    """
    from .registry import ToolRegistry

    tools, handlers = get_planning_tools(moveit, op_manager)
    registry = ToolRegistry()
    registry.register_tools(tools, handlers)
    registry.setup_server_handlers(server)
