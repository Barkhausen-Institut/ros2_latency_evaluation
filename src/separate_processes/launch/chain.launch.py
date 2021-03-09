import sys
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "separate_processes"

def generate_launch_description():
    intNodes = int(os.environ.get("INT_NODES", 1))
    pubFrequency = os.environ.get("PUB_FREQUENCY", "1")
    nodes = []
    nodes.append(Node(
                    package=PKG,
                    executable='start_node',
                    remappings=[("/start_pub_topic", "/step_0")],
                    arguments=[pubFrequency],
                    name="head"))
    for i in range(intNodes):
        remappings = []
        remappings.append(("/start_pub_topic", f"/step_{i}"))
        remappings.append(("/end_sub_topic", f"/step_{i+1}"))
        print(remappings)
        nodes.append(Node(package=PKG, executable='intermediate_node', remappings=remappings))

    nodes.append(Node(package=PKG, executable='end_node',
                      remappings=[("/end_sub_topic", f"/step_{intNodes}")]))

    return LaunchDescription(nodes)

print (sys.argv)
