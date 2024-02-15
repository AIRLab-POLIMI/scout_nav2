#!/usr/bin/env python3

# ROS2 imports
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.logging import get_logger
from rclpy.node import Node

# custom ROS2 imports
from mobile_manipulation.park_robot import RobotParking
from mobile_manipulation.robot_navigator import BasicNavigator

# python imports
import threading
from tf_transformations import euler_from_quaternion


class RobotParkingDemo(Node):

    def __init__(self, robot_parking: RobotParking):
        super().__init__('robot_parking_demo')

        self.robot_parking = robot_parking

        # create subscription to /target_pose topic
        self.create_subscription(PoseStamped, '/target_pose', self.callback_target_pose, 10)

        # run thread asynchronously
        runner_thread = threading.Thread(target=self.continous_parking_demo_thread, daemon=True)
        runner_thread.start()

    def continous_parking_demo_thread(self):
        """Continous parking demo thread

        Repeatedly calls the parking algorithm and navigation to target goal
        """
        while rclpy.ok():
            print("Parking demo thread: init procedure")
            self.robot_parking.parking_algorithm_then_navigate()

    def callback_target_pose(self, msg: PoseStamped):
        """Subscriber callback to target pose

        Parameters:
        ----------
        msg : PoseStamped
            the target pose message received from the topic

        """
        target_xyz = msg.pose.position
        # get yaw angle from quaternion
        roll, pitch, yaw = euler_from_quaternion(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        )

        self.get_logger().info("Received target pose: x = {:.3f}; y = {:.3f}; yaw = {:.3f}"
                               .format(msg.pose.position.x, msg.pose.position.y, yaw))

        # update target pose
        self.robot_parking.update_target_pose(target_xyz, yaw)


# ROS2 main function
def main(args=None):
    rclpy.init(args=args)

    # create robot parking node
    navigator = BasicNavigator()
    robot_parking = RobotParking(navigator)

    # create robot parking demo node
    robot_parking_demo = RobotParkingDemo(robot_parking)

    # create a multi-threaded executor for both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot_parking)
    executor.add_node(robot_parking_demo)

    # asynchronous spin execution of the 2 nodes
    # executor_thread = threading.Thread(target=executor.spin, daemon=True)
    # executor_thread.start()
    executor.spin()

    # end node
    robot_parking.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
