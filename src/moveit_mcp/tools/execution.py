"""Execution tool implementations."""

import logging
from typing import Any, Dict, Optional

from ..async_ops import AsyncOperationManager, OperationType
from ..moveit_wrapper import MoveItWrapper
from .errors import log_tool_error

try:
    from mcp.server import Server
    from mcp.types import Tool, TextContent
except ImportError:
    pass

logger = logging.getLogger(__name__)


def get_execution_tools(
    moveit: MoveItWrapper, op_manager: AsyncOperationManager, server_instance=None
) -> tuple[list, dict]:
    """
    Get execution tool definitions and handlers.

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
    async def execute_plan_handler(arguments: dict) -> list:
        """
        Execute a planned trajectory.

        Args (from arguments dict):
            operation_id: Operation ID of the plan to execute (from plan_to_*)
            controllers: Optional list of controller names

        Returns:
            Execution operation ID for async tracking
        """
        try:
            check_initialized()
            plan_operation_id = arguments["operation_id"]
            controllers = arguments.get("controllers")

            # Get the trajectory to execute
            trajectory = None
            if plan_operation_id:
                plan_status = await op_manager.get_operation_status(plan_operation_id)
                if plan_status is None:
                    return [TextContent(type="text", text=op_manager.not_found_text(plan_operation_id) + " Plan again.")]
                if plan_status["status"] == "completed":
                    trajectory = plan_status["result"]

            if not trajectory:
                return [
                    TextContent(
                        type="text",
                        text="No trajectory available to execute. Plan a motion first.",
                    )
                ]

            # Submit async execution operation
            async def execute_fn():
                return moveit.execute_trajectory(trajectory, controllers)

            operation_id = await op_manager.submit_operation(
                execute_fn, OperationType.EXECUTION, metadata={"controllers": controllers}
            )

            return [
                TextContent(
                    type="text",
                    text=f"Execution started. Operation ID: {operation_id}\n"
                    f"Use get_execution_status to monitor progress.",
                )
            ]

        except Exception as e:
            log_tool_error(logger, "execute_plan", e)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def get_execution_status_handler(arguments: dict) -> list:
        """
        Get the status of an execution operation.

        Args (from arguments dict):
            operation_id: Operation ID from execute_plan

        Returns:
            Execution status and progress
        """
        try:
            check_initialized()
            operation_id = arguments["operation_id"]
            status = await op_manager.get_operation_status(operation_id)

            if status is None:
                return [TextContent(type="text", text=op_manager.not_found_text(operation_id))]

            result_text = f"Operation: {operation_id}\n"
            result_text += f"Status: {status['status']}\n"
            result_text += f"Progress: {status['progress'] * 100:.1f}%\n"

            if status["status"] == "completed":
                if status["result"]:
                    result_text += "Execution completed successfully!\n"
                else:
                    result_text += "Execution failed.\n"
            elif status["status"] == "failed":
                result_text += f"Error: {status['error']}\n"
            elif status["status"] == "running":
                result_text += "Execution in progress...\n"

            return [TextContent(type="text", text=result_text)]

        except Exception as e:
            log_tool_error(logger, "get_execution_status", e)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def stop_execution_handler(arguments: dict) -> list:
        """
        Stop an ongoing execution.

        Args (from arguments dict):
            operation_id: Operation ID to stop

        Returns:
            Success status
        """
        try:
            check_initialized()
            operation_id = arguments["operation_id"]
            success = await op_manager.cancel_operation(operation_id)

            if success:
                return [
                    TextContent(
                        type="text",
                        text=f"Execution {operation_id} stop requested successfully.",
                    )
                ]
            else:
                return [
                    TextContent(
                        type="text",
                        text=f"Could not stop {operation_id}. It may have already finished.",
                    )
                ]

        except Exception as e:
            log_tool_error(logger, "stop_execution", e)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def plan_and_execute_handler(arguments: dict) -> list:
        """
        Plan and execute a motion in one operation.

        Args (from arguments dict):
            group: Planning group name
            target_type: "pose" or "joint_state" or "named_state"
            target: Target specification (depends on target_type)
            planner_id: Optional planner ID
            timeout: Planning timeout

        Returns:
            Combined operation ID for async tracking
        """
        try:
            check_initialized()
            group = arguments["group"]
            target_type = arguments["target_type"]
            target = arguments["target"]
            planner_id = arguments.get("planner_id")
            timeout = arguments.get("timeout", 5.0)

            # Submit combined async operation
            async def combined_fn():
                # Plan
                if target_type == "pose":
                    position = target["position"]
                    orientation = target["orientation"]
                    # Caller's frame, else the MoveIt planning frame
                    if "frame_id" in target:
                        frame_id = target["frame_id"]
                    else:
                        with moveit.planning_scene_monitor.read_only() as scene:
                            frame_id = scene.planning_frame
                    from geometry_msgs.msg import Pose

                    pose = Pose()
                    pose.position.x = float(position[0])
                    pose.position.y = float(position[1])
                    pose.position.z = float(position[2])
                    pose.orientation.x = float(orientation[0])
                    pose.orientation.y = float(orientation[1])
                    pose.orientation.z = float(orientation[2])
                    pose.orientation.w = float(orientation[3])

                    plan_result = moveit.plan_to_pose(
                        group_name=group,
                        target_pose=pose,
                        frame_id=frame_id,
                        planner_id=planner_id,
                        planning_time=timeout,
                    )
                elif target_type == "joint_state":
                    plan_result = moveit.plan_to_joint_state(
                        group_name=group,
                        joint_positions=target["joint_positions"],
                        planner_id=planner_id,
                        planning_time=timeout,
                    )
                elif target_type == "named_state":
                    plan_result = moveit.plan_to_named_state(
                        group_name=group, state_name=target["state_name"], planner_id=planner_id
                    )
                else:
                    raise ValueError(f"Unknown target_type: {target_type}")

                if not plan_result:
                    raise RuntimeError("Planning failed")

                # Execute
                success = moveit.execute_trajectory(plan_result)
                return success

            operation_id = await op_manager.submit_operation(
                combined_fn,
                OperationType.COMBINED,
                metadata={"group": group, "target_type": target_type},
            )

            return [
                TextContent(
                    type="text",
                    text=f"Plan and execute operation submitted. Operation ID: {operation_id}\n"
                    f"Use get_execution_status to monitor progress.",
                )
            ]

        except Exception as e:
            log_tool_error(logger, "plan_and_execute", e)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    # Define tool specifications
    tools = [
        Tool(
            name="execute_plan",
            description="Execute a planned trajectory",
            inputSchema={
                "type": "object",
                "properties": {
                    "operation_id": {
                        "type": "string",
                        "description": "Operation ID of the plan to execute (from plan_to_*)",
                    },
                    "controllers": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": "Optional controller names",
                    },
                },
                "required": ["operation_id"],
            },
        ),
        Tool(
            name="get_execution_status",
            description="Get the status of an execution operation",
            inputSchema={
                "type": "object",
                "properties": {
                    "operation_id": {
                        "type": "string",
                        "description": "Operation ID from execute_plan",
                    }
                },
                "required": ["operation_id"],
            },
        ),
        Tool(
            name="stop_execution",
            description="Stop an ongoing execution",
            inputSchema={
                "type": "object",
                "properties": {
                    "operation_id": {
                        "type": "string",
                        "description": "Operation ID to stop",
                    }
                },
                "required": ["operation_id"],
            },
        ),
        Tool(
            name="plan_and_execute",
            description="Plan and execute a motion in one operation",
            inputSchema={
                "type": "object",
                "properties": {
                    "group": {"type": "string", "description": "Planning group name"},
                    "target_type": {
                        "type": "string",
                        "enum": ["pose", "joint_state", "named_state"],
                        "description": "Type of target",
                    },
                    "target": {
                        "type": "object",
                        "description": (
                            "Target specification (structure depends on target_type).\n"
                            "  target_type='pose'        -> {position: [x,y,z], "
                            "orientation: [x,y,z,w], frame_id?: string}. "
                            "frame_id defaults to the robot's MoveIt planning frame "
                            "when omitted.\n"
                            "  target_type='joint_state' -> {joint_positions: [...]}\n"
                            "  target_type='named_state' -> {state_name: string}"
                        ),
                    },
                    "planner_id": {"type": "string", "description": "Optional planner ID"},
                    "timeout": {
                        "type": "number",
                        "description": "Planning timeout in seconds",
                    },
                },
                "required": ["group", "target_type", "target"],
            },
        ),
    ]

    # Map handlers
    handlers = {
        "execute_plan": execute_plan_handler,
        "get_execution_status": get_execution_status_handler,
        "stop_execution": stop_execution_handler,
        "plan_and_execute": plan_and_execute_handler,
    }

    return tools, handlers


def register_execution_tools(
    server: "Server", moveit: MoveItWrapper, op_manager: AsyncOperationManager
):
    """
    Register execution-related MCP tools (legacy compatibility).

    Note: This function is deprecated. Use get_execution_tools() with
    the unified ToolRegistry instead.
    """
    from .registry import ToolRegistry

    tools, handlers = get_execution_tools(moveit, op_manager)
    registry = ToolRegistry()
    registry.register_tools(tools, handlers)
    registry.setup_server_handlers(server)
