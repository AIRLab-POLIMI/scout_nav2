
// ROS2 c++ includes
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// custom action message
#include "mobile_manipulation_interfaces/action/parking.hpp"

// C++ includes
#include <future>
#include <string>

// Action client for parking and navigation action server
// calls action server robot_parking and uses its feedback and result data for processing and logging

namespace mobile_manipulation {

class ParkAndInteract : public rclcpp::Node {

	// syntactic sugar for action client and goal handle
	using ParkingAction = mobile_manipulation_interfaces::action::Parking;
	using GoalHandleParking = rclcpp_action::ClientGoalHandle<ParkingAction>;

public:
	/**
	 * @brief Constructor for ParkAndInteract class, contains action client for parking and navigation action server
	 * @param node_options options for the node, given by the launch file
	 * @extends Node class
	 */
	ParkAndInteract(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	/**
	 * @brief Callback function for receiving target pose from /target_pose topic
	 * @param msg target pose stamped message
	 */
	void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	/**
	 * @brief Main thread function that runs continuously as long as new target poses are available
	 */
	void main_thread(void);

	/**
	 * @brief Send goal to action server and setup callbacks for goal response, feedback and result services
	 */
	void send_goal_and_setup_callbacks(void);

	/**
	 * @brief Callback function for receiving goal response from action server
	 * @param goal_handle goal handle for the parking action
	 */
	void goal_response_callback(const GoalHandleParking::SharedPtr &goal_handle);

	/**
	 * @brief Callback function for receiving feedback from action server
	 * @param goal_handle goal handle for the parking action
	 * @param feedback feedback message from the action server
	 */
	void feedback_callback(GoalHandleParking::SharedPtr /*goal_handle*/,
						   const std::shared_ptr<const ParkingAction::Feedback> feedback);

	/**
	 * @brief Callback function for receiving result from action server
	 * @param result result message from the action server
	 */
	void result_callback(const GoalHandleParking::WrappedResult &result);

private:
	// Action client for parking and navigation action server
	rclcpp_action::Client<ParkingAction>::SharedPtr parking_action_client_;
	// Future for the goal handle, used to keep track of the goal final outcome
	std::shared_future<GoalHandleParking::SharedPtr> future_goal_handle_;

	rclcpp::Logger logger_ = rclcpp::get_logger("ParkAndInteract");

	// subscriber to /target_pose topic
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscriber_;
	// flag to check if target pose is available
	bool target_available;
	// computed target pose from the parking algorithm
	geometry_msgs::msg::PoseStamped::SharedPtr target_pose;

	// continuous running thread
	std::thread main_thread_;
};

} // namespace mobile_manipulation

RCLCPP_COMPONENTS_REGISTER_NODE(mobile_manipulation::ParkAndInteract)