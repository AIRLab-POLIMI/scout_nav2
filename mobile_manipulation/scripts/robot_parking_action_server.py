#!/usr/bin/env python3

# ROS2 python imports
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Point32
import tf2_geometry_msgs

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

        self.update_parking_goal_pose(goal_pose)

        # Set initial pose
        self.initial_pose = self.parking_node.get_current_pose()
        self.parking_node.navigator.setInitialPose(self.initial_pose)

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

        distance_error = feedback.distance_remaining

        # publish last feedback data to the action client
        feedback_msg = Parking.Feedback()
        feedback_msg.parking_goal = goal_pose.pose
        feedback_msg.distance_remaining = distance_error
        goal_handle.publish_feedback(feedback_msg)

        # get current robot position
        current_pose = self.parking_node.get_current_pose()

        # compute distance and heading error from the parking goal, assuming the same reference frame
        distance_error = self.parking_node.compute_distance_error(pose=current_pose, expected_pose=goal_pose)
        heading_error = self.parking_node.compute_heading_error(pose=current_pose, expected_pose=goal_pose)

        # log final feedback data
        self.get_logger().info("Distance Error = {}".format(distance_error))
        self.get_logger().info("Heading Error = {}".format(heading_error))

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

        # create result message to be sent to the action client
        result_msg = Parking.Result()
        result_msg.nav2_result = result_string
        result_msg.distance_error = distance_error
        result_msg.heading_error = heading_error
        result_msg.final_position = current_pose

        return result_msg
    
    def update_parking_goal_pose(self, goal_pose: PoseStamped):

        # transform the goal pose to the map frame
        transform = self.parking_node.get_tf(source_frame=goal_pose.header.frame_id, 
                                             target_frame="map")
        
        goal_pose_tf = tf2_geometry_msgs.do_transform_pose(goal_pose.pose, transform)

        target_xyz = goal_pose_tf.position
        target_quat = goal_pose_tf.orientation
        roll, pitch, yaw = euler_from_quaternion([target_quat.x, target_quat.y, target_quat.z, target_quat.w])

        self.get_logger().info("Server: received target: x = {:.3f}; y = {:.3f}, theta = {:.3f}"
                               .format(goal_pose_tf.position.x, goal_pose_tf.position.y, yaw))
        self.get_logger().info("Server: received target in frame_id: {}".format(goal_pose.header.frame_id))

        # update target pose in the parking algorithm
        self.parking_node.update_target_pose(target_xyz, yaw)


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
