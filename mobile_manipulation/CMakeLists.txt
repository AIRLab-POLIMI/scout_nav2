cmake_minimum_required(VERSION 3.8)
project(mobile_manipulation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies for C++ and Python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mobile_manipulation_interfaces REQUIRED)

# ----------------------- C++ -----------------------

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(park_and_interact src/park_and_interact.cpp)
ament_target_dependencies(park_and_interact
	rclcpp rclcpp_action rclcpp_components std_msgs geometry_msgs
	tf2_geometry_msgs mobile_manipulation_interfaces
)

# C++ library for ParkAndInteractNode action client
# add_library(park_and_interact SHARED src/park_and_interact.cpp)
# target_include_directories(park_and_interact PRIVATE
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>
# )

# target_compile_definitions(park_and_interact
# PRIVATE "PARK_AND_INTERACT_BUILDING_DLL"
# )

# ament_target_dependencies(park_and_interact
# rclcpp rclcpp_action rclcpp_components std_msgs geometry_msgs tf2_geometry_msgs mobile_manipulation_interfaces
# )

# rclcpp_components_register_nodes(park_and_interact
# PLUGIN "mobile_manipulation::ParkAndInteract"
# EXECUTABLE park_and_interact
# )

# Install Cpp executables
install(TARGETS park_and_interact
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

# ----------------------- Python -----------------------

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
	scripts/parking_demo.py
	DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
	scripts/robot_parking_action_server.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()