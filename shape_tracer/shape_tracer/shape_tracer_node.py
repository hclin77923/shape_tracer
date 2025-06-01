import rclpy
from rclpy.node import Node
import json
import numpy as np
import os

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander

class ShapeTracer(Node):
    def __init__(self):
        super().__init__('shape_tracer')

        self.declare_parameter('shape_file', 'config/shapes.json')
        shape_file = self.get_parameter('shape_file').get_parameter_value().string_value
        shape_path = os.path.join(
            os.path.dirname(__file__), '..', shape_file
        )

        with open(shape_path, 'r') as f:
            self.shapes = json.load(f)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("xarm7")
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)

        self.execute_shapes()

    def execute_shapes(self):
        for shape in self.shapes:
            self.trace_shape(shape)

    def trace_shape(self, shape):
        start_pos = np.array(shape["start_position"])
        rpy = shape["start_orientation_rpy"]
        quat = quaternion_from_euler(*rpy)

        base_pose = np.eye(4)
        base_pose[:3, :3] = self.euler_to_rot_matrix(*rpy)
        base_pose[:3, 3] = start_pos

        waypoints = []
        for vertex in shape["vertices"]:
            local_point = np.array([vertex[0], vertex[1], 0.0, 1.0])
            global_point = base_pose @ local_point
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = global_point[:3]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
            waypoints.append(pose)

        (plan, _) = self.group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )
        self.group.execute(plan, wait=True)

    @staticmethod
    def euler_to_rot_matrix(roll, pitch, yaw):
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        return Rz @ Ry @ Rx

def main(args=None):
    rclpy.init(args=args)
    node = ShapeTracer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
