# Mobile Manipulation tasks with AgileX Scout wheeled robot and Igus Rebel robotic arm


## Launch files

- `ros2 launch mobile_manipulation parking.launch.py`: launches the parking pose computation node. It can be used directly with RViz. Click on "2D Goal" in RViz, and then click on the map to set the target position. The parking goal pose will be computed and published on the topic `/target_goal`, which is then sent to the navigation stack and used to navigate the robot to the designed optimal parking position.

## Algorithms

### 1. Parking Algorithm

The parking algorithm is used to find the best feasible parking pose next to a target location, in such a way that the posterior part of the mobile robot
faces the target location. The algorithm is designed for parking the mobile robot in a close position to a location, where the robotic arm,
positioned in the back of the robot, can interact with.

1. given a goal pose (x y theta)
2. random sample n possible x,y positions in a circle of radius r and centered in the goal pose
3. save the n coordinates in a vector and order them by the lowest cartesian distance from the starting point
4. random sample m possible theta orientations, with pi < |theta| < pi/2
5. save the m orientations in a vector and order them by lowest distance in absolute value from pi (meaning that pi comes first and then the others
6. Create a vector with all n x m possible combinations, while respecting the previous order. Filter out the parking poses colliding with any walls or obstacles.
7. Compute the cost for all possible targets in the costmap and filter out the targets having the robot footprint with a cost higher than a set threshold t. Then sort the vector by lowest cost (ascending)
8. iterate for all possible combinations n x m until a feasible parking pose is found:

    for every parking(x,y, theta): // ordered by path cost

        viable = plan_path( kinematics, start, park)
        if (viable) save parking and planning

    - pick the viable path having the lowest path plan cost (first one in the vector of possible parkings)

9. check if a feasible path exists from the starting pose to the parking pose. Iterate the steps 1-4 until a target pose with a viable traversable path is found. If a new iteration of the algorithm is necessary, then change the value of r making it a little bit closer to the goal pose. Consider the skid steering kinematics while checking the parking paths.
10. once a viable parking pose is found, navigate the robot to the parking pose. If no viable parking poses are found, return an error.

