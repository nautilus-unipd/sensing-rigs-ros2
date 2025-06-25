cmake_minimum_required(VERSION 3.8)
project(normalizator)

# Set standards to C++ 17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Define compiling options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH "/home/ubuntu/ROS2_WSs/repo_sensing_rigs/ros2_ws/install/modem_msgs")

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(modem_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Specify include directory
include_directories(include)

# Specify executables to create
add_executable(${PROJECT_NAME}_node
    src/normalizator_node.cpp
    src/normalizator_main.cpp
    )
# Link dependencies used
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    rclcpp_lifecycle 
    std_msgs
    modem_msgs
    )
target_link_libraries(${PROJECT_NAME}_node rclcpp_lifecycle::rclcpp_lifecycle)

# Copy include directory to installation
install(DIRECTORY include/ DESTINATION include)

# Create executable
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

# Copy package file
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
# Copy config directory
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
# Copy launch directory
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
