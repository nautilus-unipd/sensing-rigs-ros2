cmake_minimum_required(VERSION 3.8)
project(sensing_capture)

# Set standards to C++ 17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Define compiling options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Specify include directory
include_directories(${OpenCV_INCLUDE_DIRS})

# Specify executables to create
add_executable(camera_node src/camera_node.cpp)
# Link dependencies used
ament_target_dependencies(camera_node
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    lifecycle_msgs
    OpenCV
)

# Create executable
install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})

# Copy package file
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
# Copy launch directory
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
# Copy config directory
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
