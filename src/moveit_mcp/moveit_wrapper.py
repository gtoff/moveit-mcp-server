"""MoveItPy wrapper for stateful robot control and planning."""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional, TYPE_CHECKING

import yaml

try:
    import rclpy
    import numpy as np
    from scipy.spatial.transform import Rotation
    from moveit.planning import MoveItPy, PlanningComponent
    from moveit.core.robot_state import RobotState
    from geometry_msgs.msg import Pose, PoseStamped
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive, Mesh
    from moveit_configs_utils import MoveItConfigsBuilder
    from ament_index_python.packages import get_package_share_directory
    import os
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    # Create placeholder types for type hints
    if TYPE_CHECKING:
        import numpy as np
        from geometry_msgs.msg import Pose
    logger = logging.getLogger(__name__)
    logger.warning("MoveIt Python bindings not available. Running in mock mode.")


logger = logging.getLogger(__name__)


def transform_matrix_to_pose(transform: "np.ndarray") -> "Pose":
    """
    Convert a 4x4 transformation matrix to a Pose message.

    Args:
        transform: 4x4 numpy array representing homogeneous transformation

    Returns:
        Pose message with position and orientation
    """
    pose = Pose()

    # Extract position (translation vector)
    pose.position.x = float(transform[0, 3])
    pose.position.y = float(transform[1, 3])
    pose.position.z = float(transform[2, 3])

    # Extract rotation matrix and convert to quaternion
    rotation_matrix = transform[:3, :3]
    rotation = Rotation.from_matrix(rotation_matrix)
    quat = rotation.as_quat()  # Returns [x, y, z, w]

    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])

    return pose


class MoveItWrapper:
    """Stateful wrapper around MoveItPy for robot control."""

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize MoveIt wrapper.

        Args:
            config: Configuration dictionary

        Raises:
            RuntimeError: If MoveIt is not available or configuration cannot be loaded

        Important:
            Before starting the MCP server, you MUST have the robot demo running:
                Terminal 1: ros2 launch moveit_resources_panda_moveit_config demo.launch.py
                Terminal 2: moveit-mcp-server-wrapper

            The MCP server needs active ROS topics (/joint_states, /tf, etc.) to initialize
            the planning scene monitor.
        """
        if not MOVEIT_AVAILABLE:
            raise RuntimeError(
                "MoveIt Python bindings (moveit_py) are not installed. "
                "Please install MoveIt2 with Python support."
            )

        self.config = config
        self.node_name = config.get("mcp_server", {}).get("node_name", "moveit_mcp_server")

        # Get robot configuration
        robot_config = config.get("robot", {})
        robot_name = robot_config.get("name", "panda")
        description_package = robot_config.get("description_package", "moveit_resources_panda_moveit_config")

        # Initialize ROS 2 if not already initialized
        if not rclpy.ok():
            logger.info("Initializing ROS 2 context")
            rclpy.init()

        # Build MoveIt configuration using MoveItConfigsBuilder (same as demo.launch.py)
        logger.info(f"Building MoveIt configuration for {robot_name}")

        try:
            # Try to find moveit_py config file in multiple locations
            moveit_cpp_yaml_path = None

            # First check our own package
            # Try robot-specific config first, then fallback to generic
            search_paths = [
                os.path.join(os.path.dirname(__file__), "..", "..", "config", f"{robot_name}_moveit_py.yaml"),
                os.path.join(os.path.dirname(__file__), "..", "..", "config", "moveit_py.yaml"),
                "/workspace/moveit2-mcp-server/config/moveit_py.yaml",
            ]

            # Also check the robot description package
            try:
                pkg_share = get_package_share_directory(description_package)
                search_paths.append(os.path.join(pkg_share, "config", "moveit_cpp.yaml"))
            except Exception:
                pass

            for path in search_paths:
                abs_path = os.path.abspath(path)
                if os.path.exists(abs_path):
                    moveit_cpp_yaml_path = abs_path
                    logger.info(f"Found moveit_py config: {moveit_cpp_yaml_path}")
                    break

            if not moveit_cpp_yaml_path:
                logger.warning("No moveit_py.yaml config found, using defaults")

            # Build MoveIt configuration
            # For standard moveit_resources packages, use moveit_resources_{robot_name}
            # For custom packages, the robot_name and package_name are separate
            builder = MoveItConfigsBuilder(robot_name, package_name=description_package)
            builder = builder.robot_description(file_path=f"config/{robot_name}.urdf.xacro")
            builder = builder.robot_description_semantic(file_path=f"config/{robot_name}.srdf")
            builder = builder.trajectory_execution(file_path="config/moveit_controllers.yaml")
            builder = builder.planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
                publish_planning_scene=True
            )
            builder = builder.planning_pipelines(pipelines=["ompl"])

            # Add moveit_cpp config if found
            if moveit_cpp_yaml_path:
                builder = builder.moveit_cpp(file_path=moveit_cpp_yaml_path)

            moveit_config = builder.to_moveit_configs()
        except Exception as e:
            logger.error(f"Failed to build MoveIt configuration: {e}")
            raise RuntimeError(f"Failed to load MoveIt configuration for {robot_name}: {e}")

        # Initialize MoveItPy with the configuration
        # In ROS 2, parameters are passed directly to the node
        logger.info(f"Initializing MoveItPy with node name: {self.node_name}")

        try:
            # Convert config to dictionary and pass as node parameters
            config_dict = moveit_config.to_dict()

            # Add use_sim_time parameter if configured
            use_sim_time = self.config.get("mcp_server", {}).get("use_sim_time", False)
            if use_sim_time:
                logger.info("Configuring MoveItPy to use simulation time")
                config_dict['use_sim_time'] = True

                # Workaround for MoveItPy bug in Jazzy with use_sim_time
                # Set QoS overrides to prevent crash
                # See: https://github.com/moveit/moveit2/issues/2690
                logger.info("Applying QoS overrides workaround for /clock topic")
                config_dict['qos_overrides'] = {
                    '/clock': {
                        'subscription': {
                            'depth': 1,
                            'history': 'keep_last',
                            'durability': 'volatile',
                            'reliability': 'best_effort'
                        }
                    }
                }

            # Debug: Print config keys to see what we have
            logger.debug(f"MoveIt config keys: {list(config_dict.keys())}")
            if 'planning_pipelines' in config_dict:
                logger.debug(f"Planning pipelines config: {config_dict['planning_pipelines']}")

            # Create MoveItPy without namespace (namespace support is buggy in Jazzy)
            self.moveit = MoveItPy(node_name=self.node_name, config_dict=config_dict)
        except RuntimeError as e:
            logger.error(f"Failed to initialize MoveItPy: {e}")
            logger.error(f"Config keys that were provided: {list(config_dict.keys()) if 'config_dict' in locals() else 'config_dict not created'}")
            raise

        # Cache planning components
        self.planning_components: Dict[str, PlanningComponent] = {}

        # Track collision objects added to the planning scene
        self._collision_object_ids: set = set()

        # Get robot model and planning scene monitor
        self.robot_model = self.moveit.get_robot_model()
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()

        logger.info("MoveItWrapper initialized successfully")

    def shutdown(self):
        """Shutdown MoveIt and cleanup resources."""
        logger.info("Shutting down MoveItWrapper")
        self.planning_components.clear()
        # MoveItPy handles its own cleanup

    def get_planning_component(self, group_name: str) -> PlanningComponent:
        """
        Get or create a planning component for a group.

        Args:
            group_name: Name of the planning group

        Returns:
            PlanningComponent instance

        Raises:
            ValueError: If group doesn't exist
        """
        if group_name not in self.planning_components:
            if not self.robot_model.has_joint_model_group(group_name):
                available = self.get_planning_groups()
                raise ValueError(
                    f"Planning group '{group_name}' not found. "
                    f"Available groups: {available}"
                )

            logger.debug(f"Creating planning component for group: {group_name}")
            self.planning_components[group_name] = self.moveit.get_planning_component(
                group_name
            )

        return self.planning_components[group_name]

    def get_planning_groups(self) -> List[str]:
        """
        Get list of available planning groups.

        Returns:
            List of planning group names
        """
        return self.robot_model.joint_model_group_names

    def get_end_effector_link(self, group_name: str) -> str:
        """
        Get the end effector link name for a planning group.

        Args:
            group_name: Planning group name

        Returns:
            End effector link name

        Note:
            For groups without a defined end effector, this returns the last link
            in the group's link model names.
        """
        joint_model_group = self.robot_model.get_joint_model_group(group_name)
        link_model_names = joint_model_group.link_model_names

        # The end effector is typically the last link in the chain
        if link_model_names:
            return link_model_names[-1]
        else:
            raise ValueError(f"No links found for planning group '{group_name}'")

    def plan_to_pose(
        self,
        group_name: str,
        target_pose: Pose,
        frame_id: str = "world",
        planner_id: Optional[str] = None,
        planning_time: Optional[float] = None,
    ) -> Optional[Any]:
        """
        Plan to a target Cartesian pose.

        Args:
            group_name: Planning group name
            target_pose: Target pose
            frame_id: Reference frame for pose
            planner_id: Optional planner ID
            planning_time: Optional planning timeout

        Returns:
            Planning result with trajectory, or None if planning failed
        """
        planning_component = self.get_planning_component(group_name)

        # Set start state to current state
        planning_component.set_start_state_to_current_state()

        # Create PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = target_pose

        # Get end effector link for this planning group
        ee_link = self.get_end_effector_link(group_name)
        logger.debug(f"Using end effector link '{ee_link}' for group '{group_name}'")

        # Set goal
        planning_component.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=ee_link)

        # Configure planner
        if planner_id:
            planning_component.set_planner_id(planner_id)

        # Plan
        logger.info(f"Planning to pose for group '{group_name}'")
        plan_result = planning_component.plan()

        if plan_result:
            logger.info(f"Planning succeeded for group '{group_name}'")
            return plan_result
        else:
            logger.warning(f"Planning failed for group '{group_name}'")
            return None

    def plan_to_joint_state(
        self,
        group_name: str,
        joint_positions: List[float],
        planner_id: Optional[str] = None,
        planning_time: Optional[float] = None,
    ) -> Optional[Any]:
        """
        Plan to a target joint state.

        Args:
            group_name: Planning group name
            joint_positions: Target joint positions
            planner_id: Optional planner ID
            planning_time: Optional planning timeout

        Returns:
            Planning result with trajectory, or None if planning failed
        """
        planning_component = self.get_planning_component(group_name)

        # Set start state to current state
        planning_component.set_start_state_to_current_state()

        # Create robot state for goal
        robot_state = RobotState(self.robot_model)
        joint_model_group = self.robot_model.get_joint_model_group(group_name)
        robot_state.set_joint_group_positions(group_name, joint_positions)

        # Set goal
        planning_component.set_goal_state(robot_state=robot_state)

        # Configure planner
        if planner_id:
            planning_component.set_planner_id(planner_id)

        # Plan
        logger.info(f"Planning to joint state for group '{group_name}'")
        plan_result = planning_component.plan()

        if plan_result:
            logger.info(f"Planning succeeded for group '{group_name}'")
            return plan_result
        else:
            logger.warning(f"Planning failed for group '{group_name}'")
            return None

    def plan_to_named_state(
        self,
        group_name: str,
        state_name: str,
        planner_id: Optional[str] = None,
    ) -> Optional[Any]:
        """
        Plan to a named robot state.

        Args:
            group_name: Planning group name
            state_name: Name of the target state (e.g., "ready", "home")
            planner_id: Optional planner ID

        Returns:
            Planning result with trajectory, or None if planning failed
        """
        planning_component = self.get_planning_component(group_name)

        # Set start state to current state
        planning_component.set_start_state_to_current_state()

        # Set goal to named state
        planning_component.set_goal_state(configuration_name=state_name)

        # Configure planner
        if planner_id:
            planning_component.set_planner_id(planner_id)

        # Plan
        logger.info(f"Planning to named state '{state_name}' for group '{group_name}'")
        plan_result = planning_component.plan()

        if plan_result:
            logger.info(f"Planning succeeded for group '{group_name}'")
            return plan_result
        else:
            logger.warning(f"Planning failed for group '{group_name}'")
            return None

    def execute_trajectory(self, trajectory: Any, controllers: Optional[List[str]] = None):
        """
        Execute a planned trajectory.

        Args:
            trajectory: Trajectory to execute (MotionPlanResponse or RobotTrajectory)
            controllers: Optional list of controller names

        Returns:
            True if execution succeeded
        """
        logger.info("Executing trajectory")

        # Use the MoveItPy object's execute method, not PlanningComponent
        if controllers is None:
            controllers = []

        # Extract RobotTrajectory from MotionPlanResponse if needed
        robot_trajectory = trajectory
        if hasattr(trajectory, 'trajectory'):
            logger.debug("Extracting trajectory from MotionPlanResponse")
            robot_trajectory = trajectory.trajectory

        success = self.moveit.execute(robot_trajectory, controllers=controllers)

        if success:
            logger.info("Trajectory execution succeeded")
        else:
            logger.warning("Trajectory execution failed")

        return success

    def get_current_state(self, group_name: str) -> Dict[str, Any]:
        """
        Get current robot state for a group.

        Args:
            group_name: Planning group name

        Returns:
            Dictionary with joint names and positions
        """
        with self.planning_scene_monitor.read_only() as scene:
            robot_state = scene.current_state
            joint_model_group = self.robot_model.get_joint_model_group(group_name)
            joint_names = joint_model_group.active_joint_model_names
            joint_positions = robot_state.get_joint_group_positions(group_name)

            return {
                "joint_names": joint_names,
                "joint_positions": list(joint_positions),
            }

    def get_current_pose(self, group_name: str, link_name: Optional[str] = None) -> Pose:
        """
        Get current end-effector pose.

        Args:
            group_name: Planning group name
            link_name: Optional link name (uses default end-effector if not specified)

        Returns:
            Current pose
        """
        with self.planning_scene_monitor.read_only() as scene:
            robot_state = scene.current_state
            joint_model_group = self.robot_model.get_joint_model_group(group_name)

            if not link_name:
                # Get the last link in the chain (tip link)
                link_names = joint_model_group.link_model_names
                if link_names:
                    link_name = link_names[-1]
                else:
                    raise ValueError(f"No links found for group '{group_name}'")

            transform = robot_state.get_global_link_transform(link_name)
            return transform_matrix_to_pose(transform)

    def compute_fk(
        self, group_name: str, joint_positions: List[float], link_name: Optional[str] = None
    ) -> Pose:
        """
        Compute forward kinematics.

        Args:
            group_name: Planning group name
            joint_positions: Joint positions
            link_name: Optional link name

        Returns:
            Computed pose
        """
        robot_state = RobotState(self.robot_model)
        joint_model_group = self.robot_model.get_joint_model_group(group_name)
        robot_state.set_joint_group_positions(group_name, joint_positions)

        if not link_name:
            # Get the last link in the chain (tip link)
            link_names = joint_model_group.link_model_names
            if link_names:
                link_name = link_names[-1]
            else:
                raise ValueError(f"No links found for group '{group_name}'")

        transform = robot_state.get_global_link_transform(link_name)
        return transform_matrix_to_pose(transform)

    def compute_ik(
        self,
        group_name: str,
        target_pose: Pose,
        timeout: float = 5.0,
        attempts: int = 10,
    ) -> Optional[List[float]]:
        """
        Compute inverse kinematics, seeded from the group's current joint positions,
        so the solver returns the solution nearest the robot's configuration.

        Args:
            group_name: Planning group name
            target_pose: Target pose
            timeout: IK timeout
            attempts: Number of attempts (not used by set_from_ik)

        Returns:
            Joint positions if IK succeeded, None otherwise
        """
        # RobotState has no copy constructor in moveit_py: copy the group's joints in.
        with self.planning_scene_monitor.read_only() as scene:
            seed_positions = scene.current_state.get_joint_group_positions(group_name)
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions(group_name, list(seed_positions))
        joint_model_group = self.robot_model.get_joint_model_group(group_name)

        # Get the last link in the chain (tip link)
        link_names = joint_model_group.link_model_names
        if link_names:
            link_name = link_names[-1]
        else:
            raise ValueError(f"No links found for group '{group_name}'")

        success = robot_state.set_from_ik(
            group_name, target_pose, link_name, timeout
        )

        if success:
            joint_positions = robot_state.get_joint_group_positions(group_name)
            return list(joint_positions)
        else:
            return None

    def check_collision(self, group_name: str, joint_positions: List[float]) -> bool:
        """
        Check if a joint state is in collision.

        Args:
            group_name: Planning group name
            joint_positions: Joint positions to check

        Returns:
            True if in collision, False otherwise
        """
        # Current state for the other joints; without update() there are no link transforms
        with self.planning_scene_monitor.read_only() as scene:
            robot_state = RobotState(self.robot_model)
            robot_state.joint_positions = scene.current_state.joint_positions
            robot_state.set_joint_group_positions(group_name, joint_positions)
            robot_state.update()
            return scene.is_state_colliding(robot_state, group_name)

    def _scene_frame(self, scene, frame_id: Optional[str]) -> str:
        """Caller's frame, else the planning frame; unknown frames are refused."""
        if not frame_id:
            return scene.planning_frame
        if not scene.knows_frame_transform(frame_id):
            raise ValueError(f"Unknown frame '{frame_id}' (planning frame: {scene.planning_frame})")
        return frame_id

    def add_collision_box(
        self, object_id: str, pose: Pose, dimensions: List[float], frame_id: Optional[str] = None
    ):
        """
        Add a box collision object to the planning scene.

        Args:
            object_id: Unique object ID
            pose: Pose of the box
            dimensions: [x, y, z] dimensions in meters
            frame_id: Reference frame (default: the planning frame)
        """
        collision_object = CollisionObject()
        collision_object.id = object_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        with self.planning_scene_monitor.read_write() as scene:
            collision_object.header.frame_id = self._scene_frame(scene, frame_id)
            scene.apply_collision_object(collision_object)

        self._collision_object_ids.add(object_id)
        logger.info(f"Added collision box '{object_id}' to planning scene")

    def add_collision_sphere(
        self, object_id: str, pose: Pose, radius: float, frame_id: Optional[str] = None
    ):
        """
        Add a sphere collision object to the planning scene.

        Args:
            object_id: Unique object ID
            pose: Center pose of the sphere
            radius: Radius in meters
            frame_id: Reference frame (default: the planning frame)
        """
        collision_object = CollisionObject()
        collision_object.id = object_id

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [radius]

        collision_object.primitives.append(sphere)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        with self.planning_scene_monitor.read_write() as scene:
            collision_object.header.frame_id = self._scene_frame(scene, frame_id)
            scene.apply_collision_object(collision_object)

        self._collision_object_ids.add(object_id)
        logger.info(f"Added collision sphere '{object_id}' to planning scene")

    def remove_collision_object(self, object_id: str):
        """
        Remove a collision object from the planning scene.

        Args:
            object_id: Object ID to remove
        """
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE

        with self.planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collision_object)

        self._collision_object_ids.discard(object_id)
        logger.info(f"Removed collision object '{object_id}' from planning scene")

    def clear_planning_scene(self):
        """Clear all collision objects from the planning scene."""
        with self.planning_scene_monitor.read_write() as scene:
            scene.remove_all_collision_objects()

        self._collision_object_ids.clear()
        logger.info("Cleared all collision objects from planning scene")

    def get_planning_scene_objects(self) -> List[str]:
        """
        Get list of collision object IDs in the planning scene.

        Returns:
            List of object IDs
        """
        return sorted(self._collision_object_ids)
