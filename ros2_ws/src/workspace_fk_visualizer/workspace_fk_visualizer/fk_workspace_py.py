#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import random
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2

# MoveIt2 Py API
from moveit_py import MoveItPy
from moveit_py.utils import conversions

class RA6AWorkspace(Node):
    def __init__(self, robot_description='robot_description', planning_group='arm'):
        super().__init__('ra6a_fk_moveit_py')
        
        # Publisher for workspace cloud
        self.pc_pub = self.create_publisher(PointCloud2, 'ra6a_workspace', 10)
        
        # Initialize MoveItPy
        self.moveitpy = MoveItPy(node=self, robot_description=robot_description)
        
        # Planning group
        self.group = self.moveitpy.get_planning_group(planning_group)
        
        # Joint limits
        self.joint_names = self.group.joint_names()
        self.joint_limits = []
        for jn in self.joint_names:
            joint = self.moveitpy.get_robot_model().get_joint(jn)
            self.joint_limits.append((joint.limit.lower, joint.limit.upper))
        
        # Timer to publish workspace
        self.create_timer(2.0, self.publish_workspace)

    def publish_workspace(self):
        points = []
        num_samples = 3000
        
        for _ in range(num_samples):
            # Random joint angles within limits
            angles = [random.uniform(low, high) for low, high in self.joint_limits]
            
            # Compute FK
            ee_pose = self.group.get_end_effector_pose(joint_positions=angles)
            ee_xyz = [ee_pose.translation.x, ee_pose.translation.y, ee_pose.translation.z]
            
            # Collision check
            if self.moveitpy.get_planning_scene().is_state_colliding(
                self.group.joint_state_to_dict(angles)
            ):
                continue  # skip points in collision
            
            points.append(ee_xyz)
        
        # Publish cloud
        header = Header()
        header.frame_id = 'base_link'
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(cloud_msg)
        self.get_logger().info(f'Published {len(points)} collision-free points')

def main(args=None):
    rclpy.init(args=args)
    node = RA6AWorkspace(planning_group='arm')  # replace 'arm' with your MoveIt2 group name
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
