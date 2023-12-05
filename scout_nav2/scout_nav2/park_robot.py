
# python imports
import numpy as np

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32, Polygon


# NAV2 control API imports
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.line_iterator import LineIterator
from nav2_simple_commander.footprint_collision_checker import FootprintCollisionChecker 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

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

class RobotParking(Node):

	# Constructor
	def __init__(self):
		super().__init__('robot_parking')
		self.subscription = self.create_subscription(PoseStamped,
			'/target_pose', self.callback_target, 0)
		self.subscription  # prevent unused variable warning

		self.navigator = BasicNavigator()

		self.checker = FootprintCollisionChecker()

		# Parameters
		self.target_radius = 0.5 # meters
		self.xy_samples = 10 # number of random samples for x,y coordinates
		self.theta_samples = 10 # number of random samples for theta orientation

		# Footprint dimensions
		self.robot_width = 0.9 # meters
		self.robot_length = 0.7 # meters
		self.footprint = Polygon()
		
		

	# Subscriber callback to target pose
	def callback_target(self, msg):
		self.get_logger().info("Received target pose:\n%s" % msg.pose)
		self.target = msg.pose

	# Main function	
	def setup_navigation(self):
		
		initial_pose = PoseStamped()
		initial_pose.header.frame_id = 'map'
		initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
		initial_pose.pose.position.x = 0.0
		initial_pose.pose.position.y = 0.0
		initial_pose.pose.orientation.x = 0.0
		initial_pose.pose.orientation.y = 0.0
		initial_pose.pose.orientation.z = 0.0
		initial_pose.pose.orientation.w = 1.0
		self.navigator.setInitialPose(initial_pose)

		#TODO: set footprint collision checker costmap
		
		# Do something depending on the return code
		result = self.navigator.getResult()
		if result == TaskResult.SUCCEEDED:
			print('Goal succeeded!')
		elif result == TaskResult.CANCELED:
			print('Goal was canceled!')
		elif result == TaskResult.FAILED:
			print('Goal failed!')
		else:
			print('Goal has an invalid return status!')

		self.navigator.lifecycleShutdown()

		exit(0)

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
			
	def sample_positions(self):
		# sample xy_samples random positions in a circle of radius target_radius centered in the target pose

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
		
	def sample_orientations(self):
		# sample theta_samples random orientations in the range pi /2 < theta < 3/2*pi
		orientation_samples = []
		for i in range(self.theta_samples):
			# sample a random angle
			theta = np.random.uniform(np.pi / 2, 3 * np.pi / 2)
			# save the orientation
			orientation_samples.append(theta)
		
		# return the sampled orientations in a vector
		return orientation_samples

	def compute_footprint_cost_at_pose(self, x, y, theta):
		# given a position and orientation, compute the footprint coordinates of the robot
		cost = self.checker.footprintCostAtPose(x, y, theta, self.footprint)
		if cost < 252:
			return cost
		
	

# ROS2 main function
def main(args=None):
	rclpy.init(args=args)

	robot_parking = RobotParking()

	try:
		rclpy.spin(robot_parking)
	except KeyboardInterrupt:
		pass

	robot_parking.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()





   
