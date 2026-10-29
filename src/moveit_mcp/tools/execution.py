"""Execution tool implementations."""

import asyncio
import logging
import time
from typing import Any, Dict, Optional

from ..async_ops import AsyncOperationManager, OperationType
from ..moveit_wrapper import MoveItWrapper
from .worker import LATCHED_TEXT, STOPPED, executing, latch, on_execute_thread, stop_latched

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

    # Store the last successful plan for execution
    # In a production system, you'd want better trajectory management
    _last_plan = {"trajectory": None}

    # Define individual tool handler functions
    async def execute_plan_handler(arguments: dict) -> list:
        """
        Execute the last planned trajectory.

        Args (from arguments dict):
            operation_id: Optional - specific plan operation ID to execute
            controllers: Optional list of controller names

        Returns:
            Execution operation ID for async tracking
        """
        try:
            check_initialized()
            if stop_latched.is_set():
                return [TextContent(type="text", text=LATCHED_TEXT)]
            # A second motion would queue behind the running one, and its stop would halt the running one
            running = await op_manager.running_execution()
            if running:
                return [TextContent(type="text", text=f"Error: execution {running} is still running; stop it or wait.")]
            plan_operation_id = arguments.get("operation_id")
            controllers = arguments.get("controllers")

            # Get the trajectory to execute
            trajectory = None
            if plan_operation_id:
                plan_status = await op_manager.get_operation_status(plan_operation_id)
                if plan_status and plan_status["status"] == "completed":
                    trajectory = plan_status["result"]
            else:
                trajectory = _last_plan["trajectory"]

            if not trajectory:
                return [
                    TextContent(
                        type="text",
                        text="No trajectory available to execute. Plan a motion first.",
                    )
                ]

            # Submit async execution operation
            async def execute_fn():
                return await on_execute_thread(moveit.execute_trajectory, trajectory, controllers)

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
            logger.error(f"Error in execute_plan: {e}", exc_info=True)
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
                return [TextContent(type="text", text=f"Operation {operation_id} not found")]

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
            logger.error(f"Error in get_execution_status: {e}", exc_info=True)
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
            status = await op_manager.get_operation_status(operation_id)
            # The motion runs on the execute thread: cancelling the task alone would leave the arm moving
            if status and status["status"] == "running" and status["type"] != "planning":
                moveit.stop_execution()
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
            logger.error(f"Error in stop_execution: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    # stop_arm and resume_arm one at a time, so a resume cannot clear the latch halfway through a stop
    arm_lock = asyncio.Lock()

    async def stop_arm_handler(arguments: dict) -> list:
        """Latch the stop, stop the running motion, and wait (at most 10 s) until nothing executes."""
        try:
            check_initialized()
            async with arm_lock:
                latch()
                running = await op_manager.running_execution()
                if running:
                    await op_manager.fail_operation(running, STOPPED)
                deadline = time.monotonic() + 10.0
                # Repeated: an execute() that started just before the latch reaches the controllers a moment later
                while executing.is_set():
                    if time.monotonic() > deadline:
                        return [TextContent(type="text", text="Error: the arm still executes 10 s after stop_arm.")]
                    moveit.stop_execution()
                    await asyncio.sleep(0.05)
            return [TextContent(type="text", text="Arm stop confirmed")]
        except Exception as e:
            logger.error(f"Error in stop_arm: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def resume_arm_handler(arguments: dict) -> list:
        """Clear the stop latch."""
        async with arm_lock:
            stop_latched.clear()
        return [TextContent(type="text", text="Arm resumed")]

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
            if stop_latched.is_set():
                return [TextContent(type="text", text=LATCHED_TEXT)]
            # A second motion would queue behind the running one, and its stop would halt the running one
            running = await op_manager.running_execution()
            if running:
                return [TextContent(type="text", text=f"Error: execution {running} is still running; stop it or wait.")]
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
                success = await on_execute_thread(moveit.execute_trajectory, plan_result)
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
            logger.error(f"Error in plan_and_execute: {e}", exc_info=True)
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
                        "description": "Optional plan operation ID to execute",
                    },
                    "controllers": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": "Optional controller names",
                    },
                },
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

    tools += [
        Tool(
            name="stop_arm",
            description=(
                "Stop the arm now and keep it stopped: ends the running motion, and refuses new plans and "
                "motions until resume_arm. Answers 'Arm stop confirmed' once nothing executes"
            ),
            inputSchema={"type": "object", "properties": {}},
        ),
        Tool(
            name="resume_arm",
            description="Allow plans and motions again after stop_arm",
            inputSchema={"type": "object", "properties": {}},
        ),
    ]

    # Map handlers
    handlers = {
        "stop_arm": stop_arm_handler,
        "resume_arm": resume_arm_handler,
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
