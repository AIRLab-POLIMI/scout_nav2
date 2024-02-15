#!/usr/bin/env python3

# ROS2 python imports
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

# custom action for parking
from mobile_manipulation_interfaces.action import Parking
from mobile_manipulation.park_robot import RobotParking
from mobile_manipulation.robot_navigator import BasicNavigator, TaskResult
from mobile_manipulation.costmap_2d import PyCostmap2D


class RobotParkingActionServer(Node):
    """ RobotParkingActionServer: action server for parking algorithm computation and navigation to target goal computed

    Given a goal pose, as PoseStamped message, the robot will compute a parking algorithm and navigate to the target goal.
    It will send feedback data including the computed parking goal and the progress of the navigation.

    Args
        Node (rclpy.node.Node): ROS2 python node

    """

    def __init__(self, robot_parking: RobotParking):
        """Initialize the RobotParkingActionServer class

        This action server initializes the parking algorithm and navigation to the target goal.
        The request message is a PoseStamped message representing the target pose, and the response message is a string
        with the final outcome of the navigation.
        """
        super().__init__('robot_parking_action_server')

        self.parking_node = robot_parking

        # Create action server with your custom messages
        self._action_server = ActionServer(
            node=self,
            action_type=Parking,
            action_name='robot_parking_action',
            execute_callback=self.execute_callback,
        )

        self.get_logger().info("Robot parking action server has been started")

    def execute_callback(self, goal_handle) -> Parking.Result:
        """Execute callback for the parking action server

        This callback is equivalent to the function `parking_algorithm_then_navigate` in the `RobotParking` class.
        The main difference is that this function is called when the action server receives a goal from the client,
        so it publishes feedback and result messages to the action client.

        Parameters:
        ----------
        goal_handle : Parking.Goal
            the goal handle received from the action client

        Returns:
        -------
        Parking.Result
            the result message to be sent to the action client

        """

        # Extraction of the target pose from the goal handle
        req = goal_handle.request
        goal_pose = req.goal_pose

        # Extract position data from target_pose
        target_xyz = goal_pose.pose.position
        roll, pitch, yaw = euler_from_quaternion(
            [goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
             goal_pose.pose.orientation.z, goal_pose.pose.orientation.w]
        )

        self.get_logger().info("Server: received target: x = {:.3f}; y = {:.3f}; yaw = {:.3f}"
                               .format(goal_pose.pose.position.x, goal_pose.pose.position.y, yaw))

        # update target pose in the parking algorithm
        self.parking_node.update_target_pose(target_xyz, yaw)

        # Set initial pose
        self.parking_node.get_initial_pose()

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # self.navigator.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        self.parking_node.navigator.waitUntilNav2Active(localizer="robot_localization")

        global_costmap = self.parking_node.navigator.getGlobalCostmap()
        costmap = PyCostmap2D(global_costmap)
        self.parking_node.checker.setCostmap(costmap)

        # compute optimal parking pose
        goal_pose = self.parking_node.parking_algorithm()

        # publish the goal pose computed from the target pose
        if goal_pose is not None:
            self.parking_node.target_goal_pub.publish(goal_pose)

            # send feedback data with the computed parking goal
            feedback_msg = Parking.Feedback()
            feedback_msg.distance_remaining = 100.0
            feedback_msg.parking_goal = goal_pose.pose
            goal_handle.publish_feedback(feedback_msg)

        else:
            # if the parking algorithm fails, send a failed result to the action client
            result_msg = Parking.Result()
            result_msg.nav2_result = "Parking algorithm failed to find a feasible solution!"
            goal_handle.abort()
            return result_msg

        # navigate to the parking pose chosen by the parking algorithm
        self.parking_node.navigator.goToPose(pose=goal_pose, behavior_tree=self.parking_node.bt_xml_path)

        # goToPose action definition
        # https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action

        i = 0
        feedback = None
        while not self.parking_node.navigator.isTaskComplete():
            # get feedback from the navigation task
            feedback = self.parking_node.navigator.getFeedback()

            # publish feedback data to the action client
            feedback_msg = Parking.Feedback()
            feedback_msg.parking_goal = goal_pose.pose
            feedback_msg.distance_remaining = feedback.distance_remaining
            goal_handle.publish_feedback(feedback_msg)

            # log feedback data every second
            if feedback and i % 10 == 0:
                self.get_logger().debug("Distance remaining = {}".format(feedback.distance_remaining))
            i = i + 1

        # publish last feedback data to the action client
        feedback_msg = Parking.Feedback()
        feedback_msg.parking_goal = goal_pose.pose
        feedback_msg.distance_remaining = feedback.distance_remaining
        goal_handle.publish_feedback(feedback_msg)

        # log final feedback data
        self.get_logger().debug("Distance remaining = {}".format(feedback.distance_remaining))

        # get result from the navigation task, and publish result to the action client
        result = self.parking_node.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            result_string = "Goal succeeded and navigation reached the objective!"
            goal_handle.succeed()
        elif result == TaskResult.CANCELED:
            result_string = "Goal navigation request was canceled!"
            goal_handle.canceled()
        elif result == TaskResult.FAILED:
            result_string = "Failed to navigate to goal position!"
            goal_handle.abort()
        else:
            result_string = "Unknown result status"
            goal_handle.abort()

        self.get_logger().info(result_string)

        result_msg = Parking.Result()
        result_msg.nav2_result = result_string

        return result_msg


def main(args=None):
    rclpy.init(args=args)

    # create the robot parking node and action server
    robot_parking = RobotParking(BasicNavigator())
    robot_parking_action_server = RobotParkingActionServer(robot_parking=robot_parking)

    # multi threaded executor to spin both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot_parking_action_server)
    executor.add_node(robot_parking)

    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
