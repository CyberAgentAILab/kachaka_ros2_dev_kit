import os
import yaml
import ament_index_python.packages
import launch
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # official joy package
    joy_linux_node = getJoyLinuxNode()

    # custom joy_controller package
    composable_node_container = ComposableNodeContainer(
        name='joy_controller_container',
        namespace='operation',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            getJoyControllerComponent(),
        ],
        output='screen',
    )
    return launch.LaunchDescription([
        joy_linux_node,
        composable_node_container,
    ])


def getJoyLinuxNode():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('joy_controller'),
        'config'
    )
    param_config = os.path.join(config_directory, 'joy_linux_param.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['joy_linux_node']['ros__parameters']
    node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        namespace='/operation/joy_linux',
        parameters=[params],
    )
    return node


def getJoyControllerComponent():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('joy_controller'),
        'config'
    )
    param_config = os.path.join(config_directory, 'joy_controller_param.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['joy_controller_node']['ros__parameters']
    component = ComposableNode(
        package='joy_controller',
        plugin='joy_controller::JoyControllerComponent',
        name='joy_controller_node',
        namespace='/operation/joy_controller',
        extra_arguments=[{
            "use_intra_process_comms": False,  # set True to activate zero copy transportaion
        }],
        parameters=[params],
    )
    return component
