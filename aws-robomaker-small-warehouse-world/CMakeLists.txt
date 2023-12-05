cmake_minimum_required(VERSION 3.8)
project(aws_robomaker_small_warehouse_world)

find_package(ament_cmake REQUIRED)

install(
	DIRECTORY launch gui worlds models maps
	DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/aws_robomaker_small_warehouse_world.dsv.in")

ament_package()
