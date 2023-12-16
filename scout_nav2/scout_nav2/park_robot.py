"""
Code to park the robot next to the target pose, according to the following algorithm:

0. given a goal pose (x y theta)
1. random sample n possible x,y positions in a circle of radius r and centered in the goal pose
2. save the n coordinates in a vector and order them by lowest cartesian distance from the starting point
2. random sample m possible theta orientations, with pi < |theta| < pi/2
3. save the m orientations in a vector and order them by lowest distance in absolute valoue from pi (meaning that pi comes first and then the others
4. Create vector with all n x m possible combinations, while respecting the previous order. Filter out the parking poses colliding with any walls or obstacles.
5. Compute the cost for all possible targets in the costmap and filter out the targets having the robot footprint with a cost higher than a set threshold t. Then sort the vector by lowest cost (ascending)
6. iterate for all possible combinations n x m until a feasible parking pose is found:

	for every parking(x,y, theta): // ordered by path cost

		viable = plan_path( kinematics, start, park)
		if (viable) save parking and planning

	- pick the viable path having the lowest path plan cost (first one in the vector of possible parkings)

7. check if a feasible path exist from the starting pose to the parking pose. Iterate the steps 1-4 until a target pose with viable traversable path is found. If a new iteration of the algorithm is necessary, then change the value of r making it a little bit closer to the goal pose. Consider the skid steering kinematics while checking the parking paths.
8. once a viable parking pose is found, navigate the robot to the parking pose. If no viable parking poses are found, return an error.

"""


# python imports
import numpy as np
import time
import threading

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32, Polygon
from tf_transformations import euler_from_quaternion
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener


# NAV2 control API imports
from scout_nav2.costmap_2d import PyCostmap2D
from scout_nav2.footprint_collision_checker import FootprintCollisionChecker
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class RobotParking(Node):

	NO_INFORMATION = 255
	LETHAL_OBSTACLE = 254
	INSCRIBED_INFLATED_OBSTACLE = 253
	MAX_NON_OBSTACLE = 252
	FREE_SPACE = 0

	# Constructor
	def __init__(self, navigator: BasicNavigator):
		super().__init__("robot_parking")

		# Subscriber to target pose topic
		self.subscription = self.create_subscription(
			PoseStamped, "/target_pose", self.callback_target, 10
		)
		self.subscription  # prevent unused variable warning

		# Publisher for the optimal target goal computed from the target pose
		self.target_goal_pub = self.create_publisher(PoseStamped, "/target_goal", 10)

		# NAV2 control API
		self.navigator = navigator
		self.checker = FootprintCollisionChecker()

		# Parameters
		self.target_radius = 0.5  # meters
		self.xy_samples = 10  # number of random samples for x,y coordinates
		self.theta_samples = 10  # number of random samples for theta orientation

		# Footprint dimensions
		self.robot_width = 0.9  # meters
		self.robot_length = 0.7  # meters
		self.footprint = Polygon()
		self.compute_footprint_polygon()

		self.target_available = False

		# Initial pose = starting point
		self.initial_pose = PoseStamped()

		# Create the TF2 transform listener
		self.robot_link_name = "mobile_robot_base_link"
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self)

		self.get_logger().info("Robot parking node started and initialized")

		# launch the parking algorithm in a separate thread
		park_thread = threading.Thread(target=self.setup_navigation, daemon=True)
		park_thread.start()

	# Subscriber callback to target pose
	def callback_target(self, msg: PoseStamped):
		self.get_logger().info("Received target pose:\n%s" % msg.pose)
		self.target = msg.pose
		self.target_available = True

	# Main function thread
	def setup_navigation(self):
		while rclpy.ok():
			# Set initial pose
			self.get_initial_pose()

			# Activate navigation, if not autostarted. This should be called after setInitialPose()
			# or this will initialize at the origin of the map and update the costmap with bogus readings.
			# If autostart, you should `waitUntilNav2Active()` instead.
			# self.navigator.lifecycleStartup()

			# Wait for navigation to fully activate, since autostarting nav2
			self.navigator.waitUntilNav2Active()

			global_costmap = self.navigator.getGlobalCostmap()
			costmap = PyCostmap2D(global_costmap)
			self.checker.setCostmap(costmap)

			# wait for first target to be available
			while not self.target_available:
				self.get_logger().info("Waiting for target pose...")
				time.sleep(0.5)
			self.target_available = False

			# compute optimal parking pose
			goal_pose = self.parking_algorithm()

			# publish the goal pose computed from the target pose
			if goal_pose is not None:
				self.target_goal_pub.publish(goal_pose)
			else:
				# iterate again with the next target candidate
				continue

			# navigate to the parking pose chosen by the parking algorithm
			self.navigator.goToPose(goal_pose)

			i = 0
			while not self.navigator.isTaskComplete():
				# Do something with the feedback
				i = i + 1
				feedback = self.navigator.getFeedback()
				if feedback and i % 10 == 0:
					self.get_logger().info(
						"Estimated time of arrival: "
						+ "{0:.2f}".format(
							Duration.from_msg(
								feedback.estimated_time_remaining
							).nanoseconds
							/ 1e9
						)
						+ " seconds."
					)

			# Do something depending on the return code
			result = self.navigator.getResult()
			if result == TaskResult.SUCCEEDED:
				print("Goal succeeded!")
			elif result == TaskResult.CANCELED:
				print("Goal was canceled!")
			elif result == TaskResult.FAILED:
				print("Goal failed!")
			else:
				print("Goal has an invalid return status!")

			# self.navigator.lifecycleShutdown()

	def parking_algorithm(self):
		# 1. random sample n possible x,y positions in a circle of radius r and centered in the goal pose

		target_candidates = np.empty((0, 3), float)  # x, y, theta

		position_samples = self.sample_positions()

		for position in position_samples:
			# 2. random sample m possible theta orientations, with pi < |theta| < pi/2
			orientation_samples = self.sample_orientations(position[0], position[1])
			for theta in orientation_samples:
				target_candidates = np.append(
					target_candidates, [[position[0], position[1], theta]], axis=0
				)

		self.get_logger().info(
			f"initial target candidates number = : {target_candidates.shape[0]}"
		)

		target_candidates, target_costs = self.filter_out_colliding_poses(target_candidates)

		self.get_logger().info(f"reduced candidates number = {target_candidates.shape[0]}")
		
		if target_candidates.shape[0] == 0:
			self.get_logger().error("No feasible parking poses found!")
			return None

		# rank the target candidates by lowest cost, lowest distance from the starting point and orientation closest to pi
		ranking_metrics = self.ranking_targets_by_cost_dist_theta(
			target_candidates, target_costs
		)

		#self.get_logger().info(f"computed target metrics:\n{ranking_metrics}")

		# sort the target candidates by the corresponding rank metric
		target_candidates = np.concatenate((target_candidates, target_costs), axis=1)
		target_candidates = np.concatenate((target_candidates, ranking_metrics), axis=1)
		#self.get_logger().info(f"shape candidates with scores = {target_candidates.shape}")

		sorted_indices = np.argsort(target_candidates[:, -1])[::-1]
		sorted_targets = target_candidates[sorted_indices]
		self.get_logger().info(f"computed sorted targets:\n{sorted_targets}")

		# iterate the first n target candidates until a feasible parking pose is found
		max_iterations = 10
		i = 0
		goal_pose = None
		while i < max_iterations:
			# use navigator to plan a path from the starting pose to the parking pose
			# and check if the path is feasible
			# if feasible, then navigate the robot to the parking pose
			# if not feasible, then iterate again with the next target candidate
			# if no feasible parking poses are found, return an error
			goal_pose = self.convert_to_pose_stamped(
				sorted_targets[i][0],
				sorted_targets[i][1],
				sorted_targets[i][2],
			)
			path = self.navigator.getPath(self.initial_pose, goal_pose)
			if path is not None:
				print("Path to goal pose is feasible! Metric rank = %f" % sorted_targets[i][-1])
				break
			else:
				print("Attempt %d in checking path feasibility result in not feasible path!" % i)
				i += 1

		if goal_pose is None:
			self.get_logger().error("No feasible paths found within 10 attempts!")

		return goal_pose

	# ---------------------------------------------------------------------------------------------
	# Helper functions
	# ---------------------------------------------------------------------------------------------

	def compute_footprint_polygon(self):
		self.footprint.points = [Point32(), Point32(), Point32(), Point32()]
		self.footprint.points[0].x = -self.robot_length / 2
		self.footprint.points[0].y = -self.robot_width / 2
		self.footprint.points[1].x = self.robot_length / 2
		self.footprint.points[1].y = -self.robot_width / 2
		self.footprint.points[2].x = self.robot_length / 2
		self.footprint.points[2].y = self.robot_width / 2
		self.footprint.points[3].x = -self.robot_length / 2
		self.footprint.points[3].y = self.robot_width / 2

	def get_initial_pose(self):
		# Wait for the listener to connect to the TF2 data
		time_now = rclpy.time.Time()
		while not self.tf_buffer.can_transform(
			"map",
			self.robot_link_name,
			rclpy.time.Time(),
			timeout=rclpy.duration.Duration(seconds=0.1),
		):
			self.get_logger().info(
				f"Waiting for transform from map to {self.robot_link_name} ..."
			)
			time.sleep(0.5)
			time_now = rclpy.time.Time()

		# Get the pose of the link frame relative to the base frame
		try:
			robot_transform = self.tf_buffer.lookup_transform(
				"map",
				self.robot_link_name,
				rclpy.time.Time(),
				timeout=rclpy.duration.Duration(seconds=0.1),
			)
		except TransformException as ex:
			self.get_logger().info(f"Could not transform tf: {ex}")
			time.sleep(0.5)

		time_now = self.navigator.get_clock().now()
		self.initial_pose.header.frame_id = "map"
		self.initial_pose.header.stamp = time_now.to_msg()
		self.initial_pose.pose.position.x = robot_transform.transform.translation.x
		self.initial_pose.pose.position.y = robot_transform.transform.translation.y
		self.initial_pose.pose.position.z = robot_transform.transform.translation.z
		self.initial_pose.pose.orientation = robot_transform.transform.rotation
		self.navigator.setInitialPose(self.initial_pose)

		# convert initial pose orientation from quaternion to euler angles
		euler = euler_from_quaternion(
			[
				self.initial_pose.pose.orientation.x,
				self.initial_pose.pose.orientation.y,
				self.initial_pose.pose.orientation.z,
				self.initial_pose.pose.orientation.w,
			]
		)

		self.get_logger().info(
			f"Initial pose set to: {self.initial_pose.pose.position.x:.3f} {self.initial_pose.pose.position.y:.3f} {euler[2]:.3f} "
		)

	# sample xy_samples random positions in a circle of radius target_radius centered in the target pose
	def sample_positions(self):
		position_samples = []
		for i in range(self.xy_samples):
			# sample a random angle
			theta = np.random.uniform(0, 2 * np.pi)
			# compute x,y coordinates
			x = self.target.position.x + self.target_radius * np.cos(theta)
			y = self.target.position.y + self.target_radius * np.sin(theta)
			# save the position
			position_samples.append([x, y])

		# return the sampled positions in a vector
		return position_samples

	def sample_orientations(self, goal_x: float, goal_y: float):
		# base angle is the angle of the vector starting from the target pose and ending in the given new goal pose
		base_angle = np.arctan2(
			goal_y - self.target.position.y, goal_x - self.target.position.x
		)

		# sample theta_samples random orientations in the range -pi /2 < theta < pi/2
		orientation_samples = np.random.uniform(
			-np.pi / 2, np.pi / 2, size=self.theta_samples
		)

		orientation_samples = np.add(orientation_samples, base_angle)

		# return the sampled orientations in a vector
		return orientation_samples

	def compute_footprint_cost_at_pose(self, x: float, y: float, theta: float):
		# given a position and orientation, compute the footprint coordinates of the robot
		cost = self.checker.footprintCostAtPose(
			x=x, y=y, theta=theta, footprint=self.footprint
		)
		if cost >= 252:
			self.get_logger().info("Footprint cost is too high: %d" % cost)
		return cost

	# convert x,y,theta coordinates to PoseStamped
	def convert_to_pose_stamped(self, x: float, y: float, theta: float):
		pose = PoseStamped()
		pose.header.frame_id = "map"
		pose.header.stamp = self.navigator.get_clock().now().to_msg()
		pose.pose.position.x = x
		pose.pose.position.y = y
		pose.pose.orientation.x = 0.0
		pose.pose.orientation.y = 0.0
		pose.pose.orientation.z = np.sin(theta / 2.0)
		pose.pose.orientation.w = np.cos(theta / 2.0)
		return pose
	
	# filter out the parking poses colliding with any walls or obstacles
	# removes the rows from the target_candidates corresponding to the colliding poses
	# returns the filtered target_candidates and the corresponding target_costs
	def filter_out_colliding_poses(self, target_candidates):
		# filter out the parking poses colliding with any walls or obstacles.
		target_costs = np.empty((0, 1), dtype=np.float)
		total = target_candidates.shape[0]
		row = 0
		while row < total:
			x = target_candidates[row][0]
			y = target_candidates[row][1]
			theta = target_candidates[row][2]
			cost = self.compute_footprint_cost_at_pose(x, y, theta)

			if cost >= self.LETHAL_OBSTACLE:  # collision detected
				target_candidates = np.delete(target_candidates, row, axis=0)
				total -= 1
			else:
				# save the cost of the targets computed
				cost = np.array([[cost]], dtype=np.float)
				target_costs = np.append(target_costs, cost, axis=0)
				row += 1
		
		return target_candidates, target_costs

	# apply ranking algorithm: maximize weighted sum of normalized cost, distance and orientation
	# lower cost means more distant from the obstacles
	# lower distance means closer to the target pose
	# lower orientation delta means closer to the base angle (more space for the robot arm to move around)
	# returns a vector of rank metrics corresponding to the target candidates
	def ranking_targets_by_cost_dist_theta(self, target_candidates, target_costs):
		weight_cost = 0.5
		weight_distance = 0.15
		weight_orientation = 0.35
		max_distance = 14.0  # meters

		# zip the target candidates with their costs
		target_candidates = np.concatenate((target_candidates, target_costs), axis=1)
		rank_metrics = np.empty((0, 1), float)

		for x, y, theta, cost in target_candidates:
			cost_norm = 1.0 - cost / 254.0  # lower cost means higher value

			distance = np.sqrt(
				(x - self.target.position.x) ** 2 + (y - self.target.position.y) ** 2
			)
			# 10 meters is the maximum distance from the target pose
			distance_norm = 1.0 - distance / max_distance

			orientation = np.arctan2(
				y - self.target.position.y, x - self.target.position.x
			)
			# delta from the base angle = direction from target pose to the new goal pose
			orientation_delta = np.abs(orientation - theta)
			# pi/2 is the maximum orientation angle delta
			orientation_norm = 1.0 - orientation_delta / (np.pi / 2.0)

			# compute the rank metric as a weighted sum of the normalized cost, distance and orientation
			rank_metric = (
				weight_cost * cost_norm
				+ weight_distance * distance_norm
				+ weight_orientation * orientation_norm
			)
			rank_metrics = np.append(rank_metrics, [[rank_metric]], axis=0)

		return rank_metrics


# ROS2 main function
def main(args=None):
	rclpy.init(args=args)

	navigator = BasicNavigator()
	robot_parking = RobotParking(navigator)

	# create a multi-threaded executor for both nodes
	executor = rclpy.executors.MultiThreadedExecutor()
	executor.add_node(robot_parking)

	# asynchronous spin execution of the 2 nodes
	# executor_thread = threading.Thread(target=executor.spin, daemon=True)
	# executor_thread.start()
	executor.spin()

	# end node
	robot_parking.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
