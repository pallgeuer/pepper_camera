cmake_minimum_required(VERSION 3.0.2)
project(pepper_camera)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	camera_info_manager
	image_transport
	sensor_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)

add_service_files(FILES
	Reconfigure.srv
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(
	INCLUDE_DIRS
	LIBRARIES
	CATKIN_DEPENDS
	DEPENDS
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${GSTREAMER_INCLUDE_DIRS}
)

add_library(pepper_camera
	src/pepper_camera.cpp
	src/pepper_camera_remote.cpp
)
add_dependencies(pepper_camera
	${pepper_camera_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

add_executable(pepper_camera_node
	src/pepper_camera_node.cpp
)
set_target_properties(pepper_camera_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(pepper_camera_node
	${pepper_camera_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pepper_camera_node
	pepper_camera
	${catkin_LIBRARIES}
	${GSTREAMER_LIBRARIES}
)
# EOF
