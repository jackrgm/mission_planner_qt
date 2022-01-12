cmake_minimum_required(VERSION 3.0.2)
project(mission_planner_qt)

set(CMAKE_MESSAGE_LOG_LEVEL STATUS)

add_compile_options(-std=c++11)

# catkin ws uses an out-of-source build setup
set(CMAKE_INCLUDE_CURRENT_DIR ON)

message(STATUS "Checking all dependencies for ${PROJECT_NAME}...")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	mission_planner
	sat_goal

	# TODO try to make it optional (omit building of certain files if not found)
	qt_robot_tasks

	# FIXME qt_robot_tasks pushing dependency management onto dependants (this)
        # the following shouldn't need to be declared here (but in qtrt)
		# there's a bug somewhere in qt_robot_tasks that fails to manage these
		# these can be removed when qt_robot_tasks manage them for us
		# causes linkage errors if removed (without qt_robot_tasks fix)
	cv_bridge
	double_ackermann
	image_transport
	robot_safety
	sensor_msgs
	telemetry
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)

message(STATUS "Setting configurations for Qt...")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(AUTOGEN_BUILD_DIR "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_autogen")
set(AUTOGEN_TARGETS_FOLDER "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_autogen")
file(MAKE_DIRECTORY ${AUTOGEN_TARGETS_FOLDER})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mission_planner_qt
	CATKIN_DEPENDS roscpp mission_planner qt_robot_tasks
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${AUTOGEN_TARGETS_FOLDER}/include
	${Qt5Gui_INCLUDE_DIRS}
	${Qt5Widgets_INCLUDE_DIRS}
)

# should header files be here? (autouic wouldn't recognise them otherwise)
add_library(${PROJECT_NAME}
	include/${PROJECT_NAME}/missionwindow.h
	include/${PROJECT_NAME}/backend.h
	src/backend.cpp
	src/mission_qt.cpp
	src/missionwindow.cpp
	src/mission_manager.cpp
	src/trail_manager.cpp
)

# TODO seperate executable for qt_robot_tasks? or roslaunch?
# with catkin_make all packages are built within a single cmake context
# recommended prefix ensures target names across catkin packages don't collide
add_executable(${PROJECT_NAME}_mission_qt
	src/mission_qt.cpp
)

# rename/remove prefix afterwards (i.e. project_name_exec_name -> exec_name)
set_target_properties(${PROJECT_NAME}_mission_qt
	PROPERTIES OUTPUT_NAME mission_qt PREFIX ""
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_mission_qt
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_mission_qt
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	Qt5::Gui
	Qt5::Widgets
)

# TODO add install config

# TODO add testing config
