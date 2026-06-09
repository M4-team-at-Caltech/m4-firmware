cmake_minimum_required(VERSION 3.12)
project(m4_firmware)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Dry execution mode
set(DRY_EXECUTION_MODE OFF CACHE BOOL "Enable dry execution mode" FORCE)

# Simulation mode (Gazebo + PX4 SITL bridge)
set(SIMULATION ON CACHE BOOL "Enable Gazebo simulation mode" FORCE)
if(DRY_EXECUTION_MODE)
    add_compile_definitions(DRY_EXECUTION_MODE)
    message(STATUS "✓ DRY_EXECUTION_MODE is ENABLED")
else()
    message(STATUS "✗ DRY_EXECUTION_MODE is DISABLED")
endif()

if(SIMULATION)
    add_compile_definitions(SIMULATION)
    message(STATUS "✓ SIMULATION mode is ENABLED")
else()
    message(STATUS "✗ SIMULATION mode is DISABLED")
endif()

# Set project root directory
add_definitions("-DPROJECT_ROOT_DIR=\"${PROJECT_SOURCE_DIR}\"")

# Modify the latency timer in dynamixel_sdk
add_definitions("-DLATENCY_TIMER=1")

add_subdirectory(third-party/yaml-cpp)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#find_package(mavros_msgs REQUIRED)

set(ROS_DEPENDENCIES rclcpp std_msgs geometry_msgs nav_msgs trajectory_msgs px4_msgs custom_msgs tf2_ros tf2_geometry_msgs)

# Find other packages
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
    include
    include/dynamixel_sdk
    third-party/yaml-cpp/include
)

# DynamixelInterface library
add_library(DynamixelInterface STATIC
    src/DynamixelInterface.cpp
    include/DynamixelInterface.h
)
target_link_libraries(DynamixelInterface PRIVATE dynamixel_sdk yaml-cpp utils)

# Utils library
add_library(utils STATIC
    src/utils.cpp
    include/utils.h
)

# Logger library
add_library(Logger STATIC
    src/Logger.cpp
    include/Logger.h
)
target_link_libraries(Logger PRIVATE utils)

# M4_base library
add_library(m4_base STATIC
    include/M4Base.h
    src/M4BaseCpp/M4Init.cpp
    src/M4BaseCpp/M4Control.cpp
    src/M4BaseCpp/M4State.cpp
    src/M4BaseCpp/M4Logging.cpp
    src/M4BaseCpp/M4CommandAPI.cpp
    src/M4BaseCpp/M4Actions.cpp
    src/M4BaseCpp/M4Helpers.cpp
    src/M4BaseCpp/M4HwBridge.cpp
)
target_link_libraries(m4_base DynamixelInterface utils Logger yaml-cpp)
if(SIMULATION)
    ament_target_dependencies(m4_base rclcpp std_msgs)
endif()

# Dynamixel SDK
add_library(dynamixel_sdk STATIC
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/group_fast_bulk_read.cpp
    src/dynamixel_sdk/group_fast_sync_read.cpp
    src/dynamixel_sdk/group_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_arduino.cpp
    src/dynamixel_sdk/port_handler_linux.cpp
    src/dynamixel_sdk/port_handler_mac.cpp
    src/dynamixel_sdk/port_handler_windows.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    include/dynamixel_sdk/dynamixel_sdk.h
    include/dynamixel_sdk/group_bulk_read.h
    include/dynamixel_sdk/group_bulk_write.h
    include/dynamixel_sdk/group_fast_bulk_read.h
    include/dynamixel_sdk/group_fast_sync_read.h
    include/dynamixel_sdk/group_handler.h
    include/dynamixel_sdk/group_sync_read.h
    include/dynamixel_sdk/group_sync_write.h
    include/dynamixel_sdk/packet_handler.h
    include/dynamixel_sdk/port_handler.h
    include/dynamixel_sdk/port_handler_arduino.h
    include/dynamixel_sdk/port_handler_linux.h
    include/dynamixel_sdk/port_handler_mac.h
    include/dynamixel_sdk/port_handler_windows.h
    include/dynamixel_sdk/protocol1_packet_handler.h
    include/dynamixel_sdk/protocol2_packet_handler.h
)

set(UNIT_TEST_SCRIPTS
    dynamixel_interface_test
	m4_base_test
)

set(SCRIPTS
	keyboard_interface
)

set(ROS_NODES
# 	m4_controller_node
)

set(COMMON_LIBRARIES
    m4_base DynamixelInterface dynamixel_sdk yaml-cpp utils Logger
)

# Common function for compilation
function(compile_executable TARGET_NAME SOURCE_FILES LIBRARIES ROS_DEPS)
    add_executable(${TARGET_NAME} ${SOURCE_FILES})
    ament_target_dependencies(${TARGET_NAME} ${ROS_DEPS})
    target_link_libraries(${TARGET_NAME} ${LIBRARIES})
endfunction()


# Unit tests and standalone scripts require real hardware constructor — skip in SIMULATION mode
if(NOT SIMULATION)
    # Set up unit tests
    foreach(SCRIPT IN LISTS UNIT_TEST_SCRIPTS)
        set(SOURCES ${PROJECT_SOURCE_DIR}/scripts/unit-tests/${SCRIPT}.cpp)
        compile_executable(${SCRIPT} "${SOURCES}" "${COMMON_LIBRARIES}" "${ROS_DEPENDENCIES}")
    endforeach()

    # Set up test scripts
    foreach(SCRIPT IN LISTS SCRIPTS)
        set(SOURCES ${PROJECT_SOURCE_DIR}/scripts/${SCRIPT}.cpp)
        compile_executable(${SCRIPT} "${SOURCES}" "${COMMON_LIBRARIES}" "${ROS_DEPENDENCIES}")
    endforeach()
endif()

# Set up ROS nodes
foreach(SCRIPT IN LISTS ROS_NODES)
    set(SOURCES ${PROJECT_SOURCE_DIR}/scripts/nodes/rc_teleop_controller/${SCRIPT}.cpp)
    compile_executable(${SCRIPT} "${SOURCES}" "${COMMON_LIBRARIES}" "${ROS_DEPENDENCIES}")
endforeach()

add_executable(m4_controller_node
    ${PROJECT_SOURCE_DIR}/scripts/nodes/rc_teleop_controller/m4_controller_node.cpp
    ${PROJECT_SOURCE_DIR}/scripts/nodes/rc_teleop_controller/m4_controller_auto.cpp
    ${PROJECT_SOURCE_DIR}/scripts/nodes/rc_teleop_controller/src/RCcontroller.cpp
)

ament_target_dependencies(m4_controller_node ${ROS_DEPENDENCIES})
target_link_libraries(m4_controller_node ${COMMON_LIBRARIES})

# Install targets
if(SIMULATION)
    install(TARGETS
        m4_controller_node
        DESTINATION lib/${PROJECT_NAME}
    )
else()
    install(TARGETS
        ${UNIT_TEST_SCRIPTS}
        ${SCRIPTS}
        ${ROS_NODES}
        m4_controller_node
        DESTINATION lib/${PROJECT_NAME}
    )
endif()

# Install launch files, config files, etc. if you have any
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()