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


#list(APPEND CMAKE_PREFIX_PATH "/home/ubuntu/ROS2_WSs/ws_sensing_rigs_v_one/install")

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)

# Specify executables to create
add_executable(norm_sub src/normalizator_sub.cpp)
ament_target_dependencies(norm_sub rclcpp custom_msgs)

# Create executable
install(TARGETS
    norm_sub
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
