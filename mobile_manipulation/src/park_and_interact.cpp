
#include "park_and_interact.hpp"

using namespace mobile_manipulation;

/**
 * @brief Constructor for ParkAndInteract class, contains action client for parking and navigation action server
 * @param node_options options for the node, given by the launch file
 * @extends Node class
*/
ParkAndInteract::ParkAndInteract(const rclcpp::NodeOptions &node_options) : Node("park_and_interact", node_options) {
	// Create action client
	this->parking_action_client_ = rclcpp_action::create_client<ParkingAction>(this, "robot_parking_action");

	// subscription to /target_pose posestamped topic to receive goal pose from rviz2
	this->target_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/target_pose", 10, std::bind(&ParkAndInteract::target_callback, this, std::placeholders::_1));

	target_available = false;

	// Start main thread
	main_thread_ = std::thread(std::bind(&ParkAndInteract::main_thread, this));
	main_thread_.detach();
}

/**
 * @brief Callback function for receiving target pose from /target_pose topic
 * @param msg target pose stamped message
*/
void ParkAndInteract::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
	target_available = true;
	target_pose = msg;
}

/**
 * @brief Main thread function that runs continuously as long as new target poses are available
*/
void ParkAndInteract::main_thread() {
	while (rclcpp::ok()) {

		while (!target_available) {
			// wait 0.2 seconds
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
		RCLCPP_INFO(logger_, "Client: target pose received");

		// target pose received
		this->send_goal_and_setup_callbacks();
		target_available = false;

	}
}

/**
 * @brief Send goal to action server and setup callbacks for goal response, feedback and result services
*/
void ParkAndInteract::send_goal_and_setup_callbacks() {
	// wait for action server to be up and running
	while (!parking_action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
		RCLCPP_INFO(logger_, "Waiting for action server to be up...");
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	// create goal message to be sent to action server
	ParkingAction::Goal goal_msg = ParkingAction::Goal();
	goal_msg.goal_pose = *this->target_pose;

	auto send_goal_options = rclcpp_action::Client<ParkingAction>::SendGoalOptions();
	// setup callbacks for goal response, feedback and result
	send_goal_options.goal_response_callback =
		std::bind(&ParkAndInteract::goal_response_callback, this, std::placeholders::_1);
	send_goal_options.feedback_callback =
		std::bind(&ParkAndInteract::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
	send_goal_options.result_callback =
		std::bind(&ParkAndInteract::result_callback, this, std::placeholders::_1);

	// send goal to action server with the given callbacks
	// use future to handle final outcome of the goal request
	future_goal_handle_ = parking_action_client_->async_send_goal(goal_msg, send_goal_options);
	
	// this future returns the goal handle, which can be used to cancel the goal
	//auto goal_handle = future_goal_handle_.get();
	//parking_action_client_->async_cancel_goal(goal_handle);

}

/**
 * @brief Callback function for goal response: goal accepted or rejected by server
 * @param goal_handle client handle for goal request 
*/
void ParkAndInteract::goal_response_callback(const GoalHandleParking::SharedPtr &goal_handle) {
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Goal was rejected by server");
	} else {
		RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
	}
}

/**
 * @brief Callback function for received and handling feedback from action server
 * @param goal_handle client handle for goal request (unused here)
 * @param feedback feedback data message from action server
*/
void ParkAndInteract::feedback_callback(GoalHandleParking::SharedPtr /*goal_handle*/,
										const std::shared_ptr<const ParkingAction::Feedback> feedback) {
	// logging feedback data
	RCLCPP_INFO(logger_, "Feedback: distance remaining = %.3f", feedback->distance_remaining);
	// convert quaternion to euler angles
	tf2::Quaternion parking_quat(feedback->parking_goal.orientation.x, feedback->parking_goal.orientation.y,
								 feedback->parking_goal.orientation.z, feedback->parking_goal.orientation.w);
	tf2::Matrix3x3 parking_mat(parking_quat);
	double roll, pitch, yaw;
	parking_mat.getRPY(roll, pitch, yaw);

	RCLCPP_INFO(logger_, "Feedback: parking position: (x,y,theta) = (%.3f, %.3f, %.3f)",
				feedback->parking_goal.position.x, feedback->parking_goal.position.y, yaw);
}

/**
 * @brief Callback function for receiving and handling result message from action server
 * @param result result data message from action server
*/
void ParkAndInteract::result_callback(const GoalHandleParking::WrappedResult &result) {
	switch (result.code) {
	case rclcpp_action::ResultCode::SUCCEEDED:
		RCLCPP_INFO(logger_, "Goal succeeded");
		break;
	case rclcpp_action::ResultCode::ABORTED:
		RCLCPP_INFO(logger_, "Goal was aborted");
		break;
	case rclcpp_action::ResultCode::CANCELED:
		RCLCPP_INFO(logger_, "Goal was canceled");
		break;
	default:
		RCLCPP_ERROR(logger_, "Unknown result code");
		break;
	}

	// log string message from result
	RCLCPP_INFO(logger_, "Result message: %s", result.result->nav2_result.c_str());
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ParkAndInteract>();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}