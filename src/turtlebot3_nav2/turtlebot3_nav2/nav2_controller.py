#!/usr/bin/env python3

"""Navigate a TurtleBot3 to random goals using the Nav2 simple commander."""

import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.executors import ExternalShutdownException

from turtlebot3_nav2.geometry import yaw_to_quaternion
from turtlebot3_nav2.goal_sampler import sample_goal


class Nav2Controller:
    """Drive a robot to random goals until the process is stopped."""

    CANCEL_GRACE_PERIOD_SEC = 5.0

    def __init__(self) -> None:
        """Create the Nav2 node and load its configuration."""
        self.navigator = BasicNavigator(node_name='nav2_controller')
        self._declare_parameters()
        self._read_parameters()

    def _declare_parameters(self) -> None:
        """Declare the parameters that configure the controller."""
        self.navigator.declare_parameter('frame_id', 'map')
        self.navigator.declare_parameter('initial_pose.x', -2.0437519550323486)
        self.navigator.declare_parameter('initial_pose.y', -0.5062426328659058)
        self.navigator.declare_parameter('initial_pose.z', 0.0)
        self.navigator.declare_parameter('initial_pose.yaw', 0.0)
        self.navigator.declare_parameter('goal_area.min_x', -1.5)
        self.navigator.declare_parameter('goal_area.max_x', 1.5)
        self.navigator.declare_parameter('goal_area.min_y', -1.5)
        self.navigator.declare_parameter('goal_area.max_y', 1.5)
        self.navigator.declare_parameter('navigation_timeout_sec', 600.0)

    def _read_parameters(self) -> None:
        """Read and validate the configured parameters."""
        get_parameter = self.navigator.get_parameter

        self._frame_id = str(get_parameter('frame_id').value)
        self._initial_pose = (
            float(get_parameter('initial_pose.x').value),
            float(get_parameter('initial_pose.y').value),
            float(get_parameter('initial_pose.z').value),
            float(get_parameter('initial_pose.yaw').value),
        )
        self._goal_min_x = float(get_parameter('goal_area.min_x').value)
        self._goal_max_x = float(get_parameter('goal_area.max_x').value)
        self._goal_min_y = float(get_parameter('goal_area.min_y').value)
        self._goal_max_y = float(get_parameter('goal_area.max_y').value)
        self._navigation_timeout = float(
            get_parameter('navigation_timeout_sec').value
        )

        if self._goal_min_x > self._goal_max_x:
            raise ValueError('goal_area.min_x must be <= goal_area.max_x')
        if self._goal_min_y > self._goal_max_y:
            raise ValueError('goal_area.min_y must be <= goal_area.max_y')
        if self._navigation_timeout <= 0.0:
            raise ValueError('navigation_timeout_sec must be greater than zero')
        if not self._frame_id:
            raise ValueError('frame_id must not be empty')

    def create_pose(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
    ) -> PoseStamped:
        """Create a pose in the configured map frame."""
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def create_random_pose(self) -> PoseStamped:
        """Create a random goal pose inside the configured goal area."""
        x, y, yaw = sample_goal(
            self._goal_min_x,
            self._goal_max_x,
            self._goal_min_y,
            self._goal_max_y,
        )
        return self.create_pose(x, y, 0.0, yaw)

    def set_initial_pose(self) -> None:
        """Set the robot's initial pose for localization."""
        x, y, z, yaw = self._initial_pose
        self.navigator.setInitialPose(self.create_pose(x, y, z, yaw))

    def navigate_to_pose(self, pose: PoseStamped) -> bool:
        """Navigate to a pose and return whether the goal succeeded."""
        logger = self.navigator.get_logger()

        if not self.navigator.goToPose(pose):
            logger.error('Nav2 rejected the goal')
            return False

        started_at = time.monotonic()
        while not self.navigator.isTaskComplete():
            if time.monotonic() - started_at >= self._navigation_timeout:
                logger.warning(
                    'Goal timed out after %.1f seconds; canceling it',
                    self._navigation_timeout,
                )
                self.navigator.cancelTask()
                cancel_deadline = time.monotonic() + self.CANCEL_GRACE_PERIOD_SEC
                while (not self.navigator.isTaskComplete()
                       and time.monotonic() < cancel_deadline):
                    pass
                if not self.navigator.isTaskComplete():
                    logger.error('Goal did not finish canceling; skipping result')
                    return False
                break

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            logger.info('Goal succeeded')
            return True
        if result == TaskResult.CANCELED:
            logger.warning('Goal was canceled')
        elif result == TaskResult.FAILED:
            logger.error('Goal failed')
        else:
            logger.warning('Goal ended with status %s', result)
        return False

    def run(self) -> None:
        """Wait for Nav2 and navigate to random goals indefinitely."""
        logger = self.navigator.get_logger()

        # Publish the initial pose before waiting so AMCL has a pose to report.
        self.set_initial_pose()
        logger.info('Waiting for Nav2 to become active')
        self.navigator.waitUntilNav2Active()

        while rclpy.ok():
            pose = self.create_random_pose()
            logger.info(
                'Navigating to (%.2f, %.2f)',
                pose.pose.position.x,
                pose.pose.position.y,
            )
            self.navigate_to_pose(pose)

    def destroy_node(self) -> None:
        """Destroy the underlying Nav2 node and its action clients."""
        self.navigator.destroy_node()


def main(args=None) -> None:
    """Initialize ROS 2, run the controller, and shut down cleanly."""
    rclpy.init(args=args)
    controller = None
    try:
        controller = Nav2Controller()
        controller.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if controller is not None:
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
