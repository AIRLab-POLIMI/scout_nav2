cmake_minimum_required(VERSION 3.8)
project(agilex_scout)

# -------------- ROS2 cmake standard declarations
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY meshes launch urdf rviz config
	DESTINATION share/${PROJECT_NAME}
)


ament_package()
