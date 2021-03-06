import sys
import os
import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "separate_processes"
NODE = "any_node"

def generate_launch_description():
    intNodes = int(os.environ.get("INT_NODES", "1"))
    pubFrequency = os.environ.get("PUB_FREQUENCY", "1")
    msgSize = os.environ.get("MSG_SIZE", "128b")
    qos = os.environ.get("QOS", "best-effort")
    duration = os.environ.get("DURATION", "10")

    noNodes = intNodes + 2

    prefix = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


    nodeArgsList = ['--publisher-frequency', pubFrequency,
                    '--no-nodes', str(noNodes),
                    '--msg-size', msgSize,
                    '--duration', duration,
                    '--qos', qos,
                    '--prefix', prefix]
    nodes = []
    nodes.append(Node(
                    package=PKG,
                    executable=NODE,
                    remappings=[("/start_pub_topic", "/step_profile_0")],
                    arguments=nodeArgsList + ['--node-index', '0'],
                    name="start_node"))
    for i in range(intNodes):
        remappings = []
        remappings.append(("/start_pub_topic", f"/step_profile_{i}"))
        remappings.append(("/end_sub_topic", f"/step_profile_{i+1}"))
        print(remappings)
        nodes.append(Node(
                    package=PKG,
                    executable=NODE,
                    remappings=remappings,
                    arguments=nodeArgsList + ['--node-index', str(i+1)],
                    name=f"intermediate_node_{i}"))

    nodes.append(Node(
                    package=PKG,
                    executable=NODE,
                    remappings=[("/end_sub_topic", f"/step_profile_{intNodes}")],
                    arguments=nodeArgsList + ['--node-index', str(noNodes-1)],
                    name="end_node"))

    return LaunchDescription(nodes)

print (sys.argv)
