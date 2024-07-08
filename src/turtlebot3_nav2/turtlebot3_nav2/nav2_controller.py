#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class Nav2Controller(Node):
    def __init__(self):
        super().__init__('nav2_controller')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
    
    def set_initial_pose(self):
        pose_stamped = self.create_pose(-2.0437519550323486, -0.5062426328659058, 0.0,
                                        0.0, 0.0, -1.8680213426256463e-06, 0.9999999999982553)
        self.navigator.setInitialPose(pose_stamped)

    def navigate_to_pose(self, pose):
        self.navigator.goToPose(pose)        

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback.navigation_time.sec > 600:
                self.navigator.cancelTask()
        print('Goal succeeded!')
        self.move()
        

    def create_pose(self, x, y, z, qx, qy, qz, qw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose
    
    def move(self):
        random_X = random.uniform(-1.5, 1.5)
        random_Y = random.uniform(-1.5, 1.5)
        random_orientation_Z = random.uniform(-1, 1)
        destination_pose = self.create_pose(random_X, random_Y, 0.0, 0.0, 0.0, random_orientation_Z, 1.0)
        self.navigate_to_pose(destination_pose)

def main(args=None):
    rclpy.init(args=args)
    navigator = Nav2Controller()  
    
    navigator.set_initial_pose()
    navigator.move()

    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
