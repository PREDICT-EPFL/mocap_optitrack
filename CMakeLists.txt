cmake_minimum_required(VERSION 3.5)
project(mocap_optitrack)

# Default to c++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dynamic_reconfigure REQUIRED)
find_package(message_generation REQUIRED)

include_directories(include
	${rclcpp_INCLUDE_DIRS}
	${tf2_ros_INCLUDE_DIRS}
	${geometry_msgs_INCLUDE_DIRS}
	${nav_msgs_INCLUDE_DIRS}
	${dynamic_reconfigure_INCLUDE_DIRS}
	${message_generation_INCLUDE_DIRS}
)

add_subdirectory(src)

rosidl_generate_interfaces(${PROJECT_NAME}
	PointArrayStamped.msg
	DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(message_runtime dynamic_reconfigure)
ament_export_include_directories(include)

install(DIRECTORY
	launch config
	DESTINATION share/${PROJECT_NAME}/
)
ament_package()



#if (CATKIN_ENABLE_TESTING)
#	find_package(roslaunch REQUIRED)
#	roslaunch_add_file_check(launch/mocap.launch)
#
#	find_package(roslint REQUIRED)
#	file(GLOB LINT_SRCS
#			src/*.cpp
#			include/*.h)
#	set(ROSLINT_CPP_OPTS "--filter=-runtime/string,-readability/casting,-runtime/references")
#	roslint_cpp(${LINT_SRCS})
#	roslint_add_test()
#endif()
