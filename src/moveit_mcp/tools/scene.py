"""Planning scene manipulation tool implementations."""

import logging
from typing import Any, Dict, Optional

from ..moveit_wrapper import MoveItWrapper

try:
    from geometry_msgs.msg import Pose
    from mcp.server import Server
    from mcp.types import Tool, TextContent
except ImportError:
    pass

logger = logging.getLogger(__name__)


def get_scene_tools(moveit: MoveItWrapper, server_instance=None) -> tuple[list, dict]:
    """
    Get planning scene tool definitions and handlers.

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
    async def add_collision_box_handler(arguments: dict) -> list:
        """
        Add a box collision object to the planning scene.

        Args (from arguments dict):
            object_id: Unique identifier for the object
            position: Center position [x, y, z] in meters
            orientation: Orientation quaternion [x, y, z, w]
            dimensions: Box dimensions [x, y, z] in meters
            frame_id: Reference frame (default: the MoveIt planning frame)

        Returns:
            Success message
        """
        try:
            check_initialized()
            object_id = arguments["object_id"]
            position = arguments["position"]
            orientation = arguments.get("orientation", [0, 0, 0, 1])
            dimensions = arguments["dimensions"]
            frame_id = arguments.get("frame_id")

            # Validate inputs
            if len(position) != 3:
                raise ValueError("Position must have 3 elements [x, y, z]")
            if len(orientation) != 4:
                raise ValueError("Orientation must have 4 elements [x, y, z, w]")
            if len(dimensions) != 3:
                raise ValueError("Dimensions must have 3 elements [x, y, z]")

            # Create pose (ensure all values are floats for ROS)
            pose = Pose()
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])
            pose.orientation.x = float(orientation[0])
            pose.orientation.y = float(orientation[1])
            pose.orientation.z = float(orientation[2])
            pose.orientation.w = float(orientation[3])

            # Add to scene
            moveit.add_collision_box(object_id, pose, dimensions, frame_id)

            return [
                TextContent(
                    type="text",
                    text=f"Successfully added collision box '{object_id}' to planning scene.",
                )
            ]

        except Exception as e:
            logger.error(f"Error in add_collision_box: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def add_collision_sphere_handler(arguments: dict) -> list:
        """
        Add a sphere collision object to the planning scene.

        Args (from arguments dict):
            object_id: Unique identifier for the object
            position: Center position [x, y, z] in meters
            radius: Sphere radius in meters
            frame_id: Reference frame (default: the MoveIt planning frame)

        Returns:
            Success message
        """
        try:
            check_initialized()
            object_id = arguments["object_id"]
            position = arguments["position"]
            radius = arguments["radius"]
            frame_id = arguments.get("frame_id")

            # Validate inputs
            if len(position) != 3:
                raise ValueError("Position must have 3 elements [x, y, z]")

            # Create pose (ensure all values are floats for ROS)
            pose = Pose()
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])
            pose.orientation.w = 1.0  # Identity orientation

            # Add to scene
            moveit.add_collision_sphere(object_id, pose, radius, frame_id)

            return [
                TextContent(
                    type="text",
                    text=f"Successfully added collision sphere '{object_id}' to planning scene.",
                )
            ]

        except Exception as e:
            logger.error(f"Error in add_collision_sphere: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def remove_collision_object_handler(arguments: dict) -> list:
        """
        Remove a collision object from the planning scene.

        Args (from arguments dict):
            object_id: ID of object to remove

        Returns:
            Success message
        """
        try:
            check_initialized()
            object_id = arguments["object_id"]
            moveit.remove_collision_object(object_id)

            return [
                TextContent(
                    type="text",
                    text=f"Successfully removed collision object '{object_id}' from planning scene.",
                )
            ]

        except Exception as e:
            logger.error(f"Error in remove_collision_object: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def clear_planning_scene_handler(arguments: dict) -> list:
        """
        Clear all collision objects from the planning scene.

        Returns:
            Success message
        """
        try:
            check_initialized()
            moveit.clear_planning_scene()
            return [
                TextContent(
                    type="text",
                    text="Successfully cleared all collision objects from planning scene.",
                )
            ]

        except Exception as e:
            logger.error(f"Error in clear_planning_scene: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def list_collision_objects_handler(arguments: dict) -> list:
        """
        List all collision objects in the planning scene.

        Returns:
            List of object IDs
        """
        try:
            check_initialized()
            objects = moveit.get_planning_scene_objects()

            if not objects:
                return [TextContent(type="text", text="No collision objects in planning scene.")]

            result_text = f"Collision objects in planning scene ({len(objects)}):\n"
            for obj_id in objects:
                result_text += f"  - {obj_id}\n"

            return [TextContent(type="text", text=result_text)]

        except Exception as e:
            logger.error(f"Error in list_collision_objects: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def check_state_collision_handler(arguments: dict) -> list:
        """
        Check if a joint state would be in collision.

        Args (from arguments dict):
            group: Planning group name
            joint_positions: Joint positions to check

        Returns:
            Collision status
        """
        try:
            check_initialized()
            group = arguments["group"]
            joint_positions = arguments["joint_positions"]

            in_collision = moveit.check_collision(group, joint_positions)

            if in_collision:
                return [
                    TextContent(
                        type="text",
                        text=f"WARNING: Joint state is in collision for group '{group}'.",
                    )
                ]
            else:
                return [
                    TextContent(
                        type="text", text=f"Joint state is collision-free for group '{group}'."
                    )
                ]

        except Exception as e:
            logger.error(f"Error in check_state_collision: {e}", exc_info=True)
            return [TextContent(type="text", text=f"Error: {str(e)}")]

    # Define tool specifications
    tools = [
        Tool(
            name="add_collision_box",
            description="Add a box collision object to the planning scene",
            inputSchema={
                "type": "object",
                "properties": {
                    "object_id": {
                        "type": "string",
                        "description": "Unique identifier for the object",
                    },
                    "position": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Center position [x, y, z] in meters",
                    },
                    "orientation": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Orientation quaternion [x, y, z, w] (default: identity)",
                    },
                    "dimensions": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Box dimensions [x, y, z] in meters",
                    },
                    "frame_id": {
                        "type": "string",
                        "description": "Reference frame (default: the MoveIt planning frame)",
                    },
                },
                "required": ["object_id", "position", "dimensions"],
            },
        ),
        Tool(
            name="add_collision_sphere",
            description="Add a sphere collision object to the planning scene",
            inputSchema={
                "type": "object",
                "properties": {
                    "object_id": {
                        "type": "string",
                        "description": "Unique identifier for the object",
                    },
                    "position": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Center position [x, y, z] in meters",
                    },
                    "radius": {"type": "number", "description": "Sphere radius in meters"},
                    "frame_id": {
                        "type": "string",
                        "description": "Reference frame (default: the MoveIt planning frame)",
                    },
                },
                "required": ["object_id", "position", "radius"],
            },
        ),
        Tool(
            name="remove_collision_object",
            description="Remove a collision object from the planning scene",
            inputSchema={
                "type": "object",
                "properties": {
                    "object_id": {"type": "string", "description": "ID of object to remove"}
                },
                "required": ["object_id"],
            },
        ),
        Tool(
            name="clear_planning_scene",
            description="Clear all collision objects from the planning scene",
            inputSchema={"type": "object", "properties": {}},
        ),
        Tool(
            name="list_collision_objects",
            description="List all collision objects in the planning scene",
            inputSchema={"type": "object", "properties": {}},
        ),
        Tool(
            name="check_state_collision",
            description="Check if a joint state would be in collision",
            inputSchema={
                "type": "object",
                "properties": {
                    "group": {"type": "string", "description": "Planning group name"},
                    "joint_positions": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Joint positions to check",
                    },
                },
                "required": ["group", "joint_positions"],
            },
        ),
    ]

    # Map handlers
    handlers = {
        "add_collision_box": add_collision_box_handler,
        "add_collision_sphere": add_collision_sphere_handler,
        "remove_collision_object": remove_collision_object_handler,
        "clear_planning_scene": clear_planning_scene_handler,
        "list_collision_objects": list_collision_objects_handler,
        "check_state_collision": check_state_collision_handler,
    }

    return tools, handlers


def register_scene_tools(server: "Server", moveit: MoveItWrapper):
    """
    Register planning scene manipulation MCP tools (legacy compatibility).

    Note: This function is deprecated. Use get_scene_tools() with
    the unified ToolRegistry instead.
    """
    from .registry import ToolRegistry

    tools, handlers = get_scene_tools(moveit)
    registry = ToolRegistry()
    registry.register_tools(tools, handlers)
    registry.setup_server_handlers(server)
