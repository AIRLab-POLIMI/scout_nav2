
// ROS2 c++ includes
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// custom actions definitions
#include "mobile_manipulation_interfaces/action/button_find.hpp"
#include "mobile_manipulation_interfaces/action/button_press.hpp"
#include "mobile_manipulation_interfaces/action/parking.hpp"

// C++ includes
#include <future>
#include <string>

// Action client for parking and navigation action server
// calls action server robot_parking and uses its feedback and result data for processing and logging

namespace mobile_manipulation {

// syntactic sugar for action clients and goal handles

// aliases for parking action server
using ParkingAction = mobile_manipulation_interfaces::action::Parking;
using GoalHandleParking = rclcpp_action::ClientGoalHandle<ParkingAction>;

// aliases for button press action server
using ButtonPresserAction = mobile_manipulation_interfaces::action::ButtonPress;
using GoalHandleButtonPresser = rclcpp_action::ClientGoalHandle<ButtonPresserAction>;

// aliases for button finder action server
using ButtonFinderAction = mobile_manipulation_interfaces::action::ButtonFind;
using GoalHandleButtonFinder = rclcpp_action::ClientGoalHandle<ButtonFinderAction>;

class ParkAndInteract : public rclcpp::Node {

public:
	/**
	 * @brief Constructor for ParkAndInteract class
	 * 		instantiates action client for parking and navigation action server
	 * 		subscribes to /target_pose topic to receive goal pose from rviz2
	 * 	    starts a thread to run main_thread function
	 * 		instatiates action client for button press and button finder action servers
	 * @param node_options options for the node, given by the launch file
	 */
	ParkAndInteract(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	/**
	 * @brief Callback function for receiving target pose from /target_pose topic
	 * @param msg target pose stamped message
	 */
	void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	/**
	 * @brief Main thread function that runs continuously as long as new target poses are available.
	 * 		Checks if target pose is available, and when it is, it sends a goal to parking action server.
	 *  	Requires subscription to /target_pose topic for receiving target pose
	 *		This function is used for testing purposes mainly, since it tests only the parking action.
	 */
	void main_thread_parking_only(void);

	/**
	 * @brief Main thread function that runs a test for the button finder action server
	 * 		The target pose returned as a result is published to /target_pose topic for visualization in rviz2
	 * 		The feedback from the action server is also logged
	 */
	void main_thread_button_finder(void);

	/**
	 * @brief Main thread function that runs a test for the button presser action server
	 * 		The result of the action server (movement success percentage) is logged
	 * 		The feedback from the action server is also logged
	 */
	void main_thread_button_presser(void);

	/**
	 * @brief Main thread function that runs the entire demo setup.
	 * 		First it sends a goal to the button finder action server, which returns the target pose as a result.
	 * 		The second step is to send a goal to the parking action server, with the obtained target pose.
	 * 		The third step is to send a goal to the button presser action server, once the robot has reached the parking pose.
	 * 		All the steps are executed in sequence, and the result of each step is used to trigger the next step.
	 * 		Each step assumes that the previous step has been completed successfully.
	 */
	void main_thread_demo(void);

	/**
	 * @brief Send goal to parking action server and setup callbacks for goal response, feedback and result
	 * @return future for the goal handle, used to keep track of the goal final outcome
	 */
	std::shared_future<GoalHandleParking::SharedPtr> sendParkingGoal(void);

	/**
	 * @brief Callback function for receiving goal response from action server
	 * @param goal_handle goal handle for the parking action
	 */
	void parkingGoalResponseCallback(const GoalHandleParking::SharedPtr &goal_handle);

	/**
	 * @brief Callback function for receiving feedback from parking action server
	 * @param goal_handle goal handle for the parking action
	 * @param feedback feedback message from the action server
	 */
	void parkingFeedbackCallback(GoalHandleParking::SharedPtr /*goal_handle*/,
								 const std::shared_ptr<const ParkingAction::Feedback> feedback);

	/**
	 * @brief Callback function for receiving result from parking action server
	 * @param result result message from the action server
	 */
	void parkingResultCallback(const GoalHandleParking::WrappedResult &result);

	/**
	 * @brief Send goal to button finder action server and setup callbacks for goal response, feedback and result
	 * @return future for the goal handle, used to keep track of the goal final outcome
	 */
	std::shared_future<GoalHandleButtonFinder::SharedPtr> sendButtonFinderGoal(void);

	/**
	 * @brief Callback function for receiving goal response from button finder action server
	 * @param goal_handle goal handle for the button finder action
	 */
	void buttonFinderGoalResponseCallback(const GoalHandleButtonFinder::SharedPtr &goal_handle);

	/**
	 * @brief Callback function for receiving feedback from button finder action server
	 * @param goal_handle goal handle for the button finder action
	 * @param feedback feedback message from the action server
	 */
	void buttonFinderFeedbackCallback(GoalHandleButtonFinder::SharedPtr /*goal_handle*/,
									  const std::shared_ptr<const ButtonFinderAction::Feedback> feedback);

	/**
	 * @brief Callback function for receiving result from button finder action server
	 * @param result result message from the action server
	 */
	void buttonFinderResultCallback(const GoalHandleButtonFinder::WrappedResult &result);

	/**
	 * @brief Send goal to button presser action server and setup callbacks for goal response, feedback and result
	 * @return future for the goal handle, used to keep track of the goal final outcome
	 */
	std::shared_future<GoalHandleButtonPresser::SharedPtr> sendButtonPresserGoal(void);

	/**
	 * @brief Callback function for receiving goal response from button presser action server
	 * @param goal_handle goal handle for the button presser action
	 */
	void buttonPresserGoalResponseCallback(const GoalHandleButtonPresser::SharedPtr &goal_handle);

	/**
	 * @brief Callback function for receiving feedback from button presser action server
	 * @param goal_handle goal handle for the button presser action
	 * @param feedback feedback message from the action server
	 */
	void buttonPresserFeedbackCallback(GoalHandleButtonPresser::SharedPtr /*goal_handle*/,
									   const std::shared_ptr<const ButtonPresserAction::Feedback> feedback);

	/**
	 * @brief Callback function for receiving result from button presser action server
	 * @param result result message from the action server
	 */
	void buttonPresserResultCallback(const GoalHandleButtonPresser::WrappedResult &result);

private:
	// Action client for parking and navigation action server
	rclcpp_action::Client<ParkingAction>::SharedPtr parking_action_client_;

	// Action client for button press action server
	rclcpp_action::Client<ButtonPresserAction>::SharedPtr button_presser_action_client_;

	// action client for button finder action server
	rclcpp_action::Client<ButtonFinderAction>::SharedPtr button_finder_action_client_;

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