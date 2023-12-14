cmake_minimum_required(VERSION 3.5)
project(nav2_bringup_custom)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	if("cxx_std_17" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
		set(CMAKE_CXX_STANDARD 17)
	else()
		message(FATAL_ERROR "cxx_std_17 could not be found.")
	endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
	add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
