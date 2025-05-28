from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    package_name = "cv_algorithms"

    # Retrieve C.V. algorithms parameters from their configuration files
    param_mono_ir = PathJoinSubstitution([FindPackageShare(package_name), "config", "mono_ir_config.yaml"])
    param_stereo_ir = PathJoinSubstitution([FindPackageShare(package_name), "config", "stereo_ir_config.yaml"])
    param_stereo_vo = PathJoinSubstitution([FindPackageShare(package_name), "config", "stereo_vo_config.yaml"])

    # Create and configure every node parameters
    launch_mono_ir = LifecycleNode(
        package = package_name,
        executable = "mono_ir_node",
        name = "mono_ir_node",
        namespace = package_name,
        parameters = [param_mono_ir],
        output = "log",
        autostart = True
    )
    launch_stereo_ir = LifecycleNode(
        package = package_name,
        executable = "stereo_ir_node",
        name = "stereo_ir_node",
        namespace = package_name,
        parameters = [param_stereo_ir],
        output = "log",
        autostart = True
    )
    launch_stereo_vo = LifecycleNode(
        package = package_name,
        executable = "stereo_vo_node",
        name = "stereo_vo_node",
        namespace = package_name,
        parameters = [param_stereo_vo],
        output = "log",
        autostart = True
    )

    # Return the nodes
    return LaunchDescription([
        launch_mono_ir,
        launch_stereo_ir,
        launch_stereo_vo
    ])
