from os import path

import yaml

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration

def load_yaml(yaml_file_path):
    with open(yaml_file_path,'r') as f:
        return yaml.load(f)

def launch_setup(context, *args, **kwargs):
    
    parameter_yaml_file  = LaunchConfiguration('param_yaml_file').perform(context)
    parameters_yaml = load_yaml(parameter_yaml_file)
    
    print(str(parameters_yaml))
    
    
    parameter_server_node = Node(
        node_executable = 'server',
        package = 'parameter_server',
        node_name = 'global_parameter_server',
        output = 'screen',
        parameters = [parameters_yaml],
        respawn_delay = 5.0)
    
    example_parameter_client_node = Node(
        node_executable = 'example_parameter_client.py',
        package = 'rclpy_parameter_utils',
        node_name = 'example_parameter_client',
        output = 'screen')
    
    return [parameter_server_node,]
            #example_parameter_client_node]

def generate_launch_description():
    
    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument('param_yaml_file',
        default_value = path.join(get_package_share_directory('rclpy_parameter_utils'), 'launch','example_params.yaml')))
    ld.add_action(OpaqueFunction(function = launch_setup))
    
    return ld
    
    