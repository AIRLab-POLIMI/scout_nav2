
#include "park_and_interact.hpp"

using namespace mobile_manipulation;

/**
 * @brief Constructor for ParkAndInteract class
 * 		instantiates action client for parking and navigation action server
 * 		subscribes to /target_pose topic to receive goal pose from rviz2
 * 	    starts a thread to run main_thread function
 * 		instatiates action client for button press and button finder action servers
 * @param node_options options for the node, given by the launch file
 */
ParkAndInteract::ParkAndInteract(const rclcpp::NodeOptions &node_options) : Node("park_and_interact", node_options) {
	// Create action client for parking action server
	this->parking_action_client_ = rclcpp_action::create_client<ParkingAction>(this, "robot_parking_action");
	// Create action client for button press action server
	this->button_presser_action_client_ = rclcpp_action::create_client<ButtonPresserAction>(this, "button_presser_action");
	// Create action client for button finder action server
	this->button_finder_action_client_ = rclcpp_action::create_client<ButtonFinderAction>(this, "button_finder_action");

	// subscription to /target_pose posestamped topic to receive goal pose from rviz2
	this->target_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/target_pose", 10, std::bind(&ParkAndInteract::target_callback, this, std::placeholders::_1));

	target_available = false;

	// Start main thread
	// NOTE: choose which main function to execute for testing purposes
	main_thread_ = std::thread(std::bind(&ParkAndInteract::main_thread_parking_only, this));
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
 * @brief Main thread function that runs continuously as long as new target poses are available.
 * 		Checks if target pose is available, and when it is, it sends a goal to parking action server.
 *  	Requires subscription to /target_pose topic for receiving target pose
 *		This function is used for testing purposes mainly, since it tests only the parking action.
 */
void ParkAndInteract::main_thread_parking_only() {
	RCLCPP_INFO(logger_, "Initializing main_thread_parking_only");
	// sleep 5 seconds
	std::this_thread::sleep_for(std::chrono::seconds(5));

	while(!target_available) {
		// wait for target pose to be available before proceeding
		// target pose is received from /target_pose topic in this case
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	// send goal to parking action server
	auto future_park_goal = sendParkingGoal();

	// wait for future to complete (goal result to be available)
	future_park_goal.wait();
	auto goal_handle = future_park_goal.get();
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Goal was rejected by server");
		return;
	}

	auto result_future = parking_action_client_->async_get_result(goal_handle);
	result_future.wait();
	auto result = result_future.get();
	if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(logger_, "Goal failed");
		return;
	}

	// log string message from result
	RCLCPP_INFO(logger_, "Parking demo terminated successfully");
}

/**
 * @brief Main thread function that runs a test for the button finder action server
 * 		The target pose returned as a result is published to /target_pose topic for visualization in rviz2
 * 		The feedback from the action server is also logged
 */
void ParkAndInteract::main_thread_button_finder(void) {
	RCLCPP_INFO(logger_, "Initializing main_thread_button_finder");
	// sleep 5 seconds
	std::this_thread::sleep_for(std::chrono::seconds(5));
	// send goal to button finder action server
	auto future_finder_goal = sendButtonFinderGoal();

	// wait for future to complete (goal result to be available)
	future_finder_goal.wait();
	auto goal_handle = future_finder_goal.get();
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Goal was rejected by server");
		return;
	}

	auto result_future = button_finder_action_client_->async_get_result(goal_handle);
	result_future.wait();
	auto result = result_future.get();
	if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(logger_, "Goal failed");
		return;
	}

	// target pose received as result
	if (target_available) {
		// publish target pose to /target_pose topic
		auto target_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
		target_publisher->publish(*target_pose);
	}
	RCLCPP_INFO(logger_, "Button finder demo terminated successfully");
}

/**
 * @brief Main thread function that runs a test for the button presser action server
 * 		The result of the action server (movement success percentage) is logged
 * 		The feedback from the action server is also logged
 */
void ParkAndInteract::main_thread_button_presser(void) {
	RCLCPP_INFO(logger_, "Initializing main_thread_button_presser");
	// sleep 5 seconds
	std::this_thread::sleep_for(std::chrono::seconds(5));
	// send goal to button finder action server
	auto future_presser_goal = sendButtonPresserGoal();

	// wait for future to complete (goal result to be available)
	future_presser_goal.wait();
	auto goal_handle = future_presser_goal.get();
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Goal was rejected by server");
		return;
	}

	auto result_future = button_presser_action_client_->async_get_result(goal_handle);
	result_future.wait();
	auto result = result_future.get();
	if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(logger_, "Goal failed");
		return;
	}

	// log result of the action server
	RCLCPP_INFO(logger_, "Button presser demo terminated successfully");
}

/**
 * @brief Main thread function that runs the entire demo setup.
 * 		First it sends a goal to the button finder action server, which returns the target pose as a result.
 * 		The second step is to send a goal to the parking action server, with the obtained target pose.
 * 		The third step is to send a goal to the button presser action server, once the robot has reached the parking pose.
 * 		All the steps are executed in sequence, and the result of each step is used to trigger the next step.
 * 		Each step assumes that the previous step has been completed successfully.
 */
void ParkAndInteract::main_thread_demo(void) {
	RCLCPP_INFO(logger_, "Initializing main_thread_demo");
	// sleep 5 seconds
	std::this_thread::sleep_for(std::chrono::seconds(5));

	// STEP 1: send goal to button finder action server
	auto future_finder_goal = sendButtonFinderGoal();

	// wait for future to complete (goal result to be available)
	future_finder_goal.wait();
	auto finder_goal_handle = future_finder_goal.get();
	if (!finder_goal_handle) {
		RCLCPP_ERROR(logger_, "Button finder goal was rejected by server");
		return;
	}

	auto finder_result_future = button_finder_action_client_->async_get_result(finder_goal_handle);
	finder_result_future.wait();
	auto result = finder_result_future.get();
	if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(logger_, "Button finder action failed");
		return;
	}

	while (!target_available) {
		// ensure target pose is available before proceeding
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	// STEP 2: send goal to parking action server with the obtained target pose
	auto future_park_goal = sendParkingGoal();

	// wait for future to complete (goal result to be available)
	future_park_goal.wait();
	auto park_goal_handle = future_park_goal.get();
	if (!park_goal_handle) {
		RCLCPP_ERROR(logger_, "Parking goal was rejected by server");
		return;
	}

	auto result_future_park = parking_action_client_->async_get_result(park_goal_handle);
	result_future_park.wait();
	auto result_park = result_future_park.get();
	if (result_park.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(logger_, "Parking goal failed");
		return;
	}

	// STEP3: send goal to button presser action server
	auto future_presser_goal = sendButtonPresserGoal();

	// wait for future to complete (goal result to be available)
	future_presser_goal.wait();
	auto presser_goal_handle = future_presser_goal.get();
	if (!presser_goal_handle) {
		RCLCPP_ERROR(logger_, "Presser goal was rejected by server");
		return;
	}

	auto result_future_presser = button_presser_action_client_->async_get_result(presser_goal_handle);
	result_future_presser.wait();
	auto result_presser = result_future_presser.get();
	if (result_presser.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(logger_, "Button presser goal failed");
		return;
	}

	RCLCPP_INFO(logger_, "Entire demo terminated successfully!");
}

/**
 * @brief Send goal to parking action server and setup callbacks for goal response, feedback and result
 * @return future for the goal handle, used to keep track of the goal final outcome
 */
std::shared_future<GoalHandleParking::SharedPtr> ParkAndInteract::sendParkingGoal(void) {
	// wait for action server to be up and running
	while (!parking_action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
		RCLCPP_INFO(logger_, "Waiting for parking action server to be up...");
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	// create goal message to be sent to action server
	ParkingAction::Goal goal_msg = ParkingAction::Goal();
	goal_msg.goal_pose = *this->target_pose;

	auto send_goal_options = rclcpp_action::Client<ParkingAction>::SendGoalOptions();
	// setup callbacks for goal response, feedback and result
	send_goal_options.goal_response_callback =
		std::bind(&ParkAndInteract::parkingGoalResponseCallback, this, std::placeholders::_1);
	send_goal_options.feedback_callback =
		std::bind(&ParkAndInteract::parkingFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
	send_goal_options.result_callback =
		std::bind(&ParkAndInteract::parkingResultCallback, this, std::placeholders::_1);

	// send goal to action server with the given callbacks
	// use future to handle final outcome of the goal request
	std::shared_future<GoalHandleParking::SharedPtr> future_park_goal_handle =
		parking_action_client_->async_send_goal(goal_msg, send_goal_options);

	// this future returns the goal handle, which can be used to cancel the goal
	// auto goal_handle = future_goal_handle_.get();
	// parking_action_client_->async_cancel_goal(goal_handle);
	return future_park_goal_handle;
}

/**
 * @brief Callback function for receiving goal response from action server
 * @param goal_handle goal handle for the parking action
 */
void ParkAndInteract::parkingGoalResponseCallback(const GoalHandleParking::SharedPtr &goal_handle) {
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Parking goal was rejected by server");
	} else {
		RCLCPP_INFO(logger_, "Parking goal accepted by server, waiting for result");
	}
}

/**
 * @brief Callback function for receiving feedback from parking action server
 * @param goal_handle goal handle for the parking action
 * @param feedback feedback message from the action server
 */
void ParkAndInteract::parkingFeedbackCallback(GoalHandleParking::SharedPtr /*goal_handle*/,
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
 * @brief Callback function for receiving result from parking action server
 * @param result result message from the action server
 */
void ParkAndInteract::parkingResultCallback(const GoalHandleParking::WrappedResult &result) {
	switch (result.code) {
	case rclcpp_action::ResultCode::SUCCEEDED:
		RCLCPP_INFO(logger_, "Parking succeeded");
		break;
	case rclcpp_action::ResultCode::ABORTED:
		RCLCPP_INFO(logger_, "Parking goal was aborted");
		return;
	case rclcpp_action::ResultCode::CANCELED:
		RCLCPP_INFO(logger_, "Parking goal was canceled");
		return;
	default:
		RCLCPP_ERROR(logger_, "Unknown parking result code");
		return;
	}

	// log string message from result
	RCLCPP_INFO(logger_, "Result message: %s", result.result->nav2_result.c_str());
	geometry_msgs::msg::Quaternion q = result.result->final_position.pose.orientation;
	float theta = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
	RCLCPP_INFO(logger_, "Final position: (x,y,theta) = (%.3f, %.3f, %.3f)",
				result.result->final_position.pose.position.x,
				result.result->final_position.pose.position.y,
				theta);
	RCLCPP_INFO(logger_, "Distance error: %.3f", result.result->distance_error);
	RCLCPP_INFO(logger_, "Heading error: %.3f", result.result->heading_error);
}

/**
 * @brief Send goal to button finder action server and setup callbacks for goal response, feedback and result
 * @return future for the goal handle, used to keep track of the goal final outcome
 */
std::shared_future<GoalHandleButtonFinder::SharedPtr> ParkAndInteract::sendButtonFinderGoal(void) {
	// wait for action server to be up and running
	while (!button_finder_action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
		RCLCPP_INFO(logger_, "Waiting for button finder action server to be up...");
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	// create goal message to be sent to action server
	ButtonFinderAction::Goal goal_msg = ButtonFinderAction::Goal();

	auto send_goal_options = rclcpp_action::Client<ButtonFinderAction>::SendGoalOptions();
	// setup callbacks for goal response, feedback and result
	send_goal_options.goal_response_callback =
		std::bind(&ParkAndInteract::buttonFinderGoalResponseCallback, this, std::placeholders::_1);
	send_goal_options.feedback_callback =
		std::bind(&ParkAndInteract::buttonFinderFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
	send_goal_options.result_callback =
		std::bind(&ParkAndInteract::buttonFinderResultCallback, this, std::placeholders::_1);

	// send goal to action server with the given callbacks
	// use future to handle final outcome of the goal request
	std::shared_future<GoalHandleButtonFinder::SharedPtr> future_finder_goal_handle =
		button_finder_action_client_->async_send_goal(goal_msg, send_goal_options);
	return future_finder_goal_handle;
}

/**
 * @brief Callback function for receiving goal response from button finder action server
 * @param goal_handle goal handle for the button finder action
 */
void ParkAndInteract::buttonFinderGoalResponseCallback(const GoalHandleButtonFinder::SharedPtr &goal_handle) {
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Button finder goal was rejected by server");
	} else {
		RCLCPP_INFO(logger_, "Button finder goal accepted by server, waiting for result");
	}
}

/**
 * @brief Callback function for receiving feedback from button finder action server
 * @param goal_handle goal handle for the button finder action
 * @param feedback feedback message from the action server
 */
void ParkAndInteract::buttonFinderFeedbackCallback(GoalHandleButtonFinder::SharedPtr /*goal_handle*/,
												   const std::shared_ptr<const ButtonFinderAction::Feedback> feedback) {
	// logging feedback data
	RCLCPP_INFO(logger_, "Feedback: %s", feedback->status.c_str());
}

/**
 * @brief Callback function for receiving result from button finder action server
 * @param result result message from the action server
 */
void ParkAndInteract::buttonFinderResultCallback(const GoalHandleButtonFinder::WrappedResult &result) {
	RCLCPP_INFO(logger_, "Result: target acquired. x = %.3f, y = %.3f, z = %.3f", result.result->target.pose.position.x,
				result.result->target.pose.position.y, result.result->target.pose.position.z);
	RCLCPP_INFO(logger_, "Result: target frame_id = %s", result.result->target.header.frame_id.c_str());
	target_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(result.result->target);
	target_available = true;
}

/**
 * @brief Send goal to button presser action server and setup callbacks for goal response, feedback and result
 * @return future for the goal handle, used to keep track of the goal final outcome
 */
std::shared_future<GoalHandleButtonPresser::SharedPtr> ParkAndInteract::sendButtonPresserGoal(void) {
	// wait for action server to be up and running
	while (!button_presser_action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
		RCLCPP_INFO(logger_, "Waiting for button presser action server to be up...");
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	// create goal message to be sent to action server
	ButtonPresserAction::Goal goal_msg = ButtonPresserAction::Goal();

	auto send_goal_options = rclcpp_action::Client<ButtonPresserAction>::SendGoalOptions();
	// setup callbacks for goal response, feedback and result
	send_goal_options.goal_response_callback =
		std::bind(&ParkAndInteract::buttonPresserGoalResponseCallback, this, std::placeholders::_1);
	send_goal_options.feedback_callback =
		std::bind(&ParkAndInteract::buttonPresserFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
	send_goal_options.result_callback =
		std::bind(&ParkAndInteract::buttonPresserResultCallback, this, std::placeholders::_1);

	// send goal to action server with the given callbacks
	// use future to handle final outcome of the goal request

	std::shared_future<GoalHandleButtonPresser::SharedPtr> future_presser_goal_handle =
		button_presser_action_client_->async_send_goal(goal_msg, send_goal_options);
	return future_presser_goal_handle;
}

/**
 * @brief Callback function for receiving goal response from button presser action server
 * @param goal_handle goal handle for the button presser action
 */
void ParkAndInteract::buttonPresserGoalResponseCallback(const GoalHandleButtonPresser::SharedPtr &goal_handle) {
	if (!goal_handle) {
		RCLCPP_ERROR(logger_, "Button presser goal was rejected by server");
	} else {
		RCLCPP_INFO(logger_, "Button presser goal accepted by server, waiting for result");
	}
}

/**
 * @brief Callback function for receiving feedback from button presser action server
 * @param goal_handle goal handle for the button presser action
 * @param feedback feedback message from the action server
 */
void ParkAndInteract::buttonPresserFeedbackCallback(GoalHandleButtonPresser::SharedPtr /*goal_handle*/,
													const std::shared_ptr<const ButtonPresserAction::Feedback> feedback) {
	// logging feedback data
	RCLCPP_INFO(logger_, "Feedback: %s", feedback->status.c_str());
}

/**
 * @brief Callback function for receiving result from button presser action server
 * @param result result message from the action server
 */
void ParkAndInteract::buttonPresserResultCallback(const GoalHandleButtonPresser::WrappedResult &result) {
	int n_goals_completed = result.result->n_goals_completed;
	float percent_completion = result.result->percent_completion;
	RCLCPP_INFO(logger_, "Result: %d goals completed out of 11; %.2f percent completion in linear motions",
				n_goals_completed, percent_completion);
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ParkAndInteract>();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}