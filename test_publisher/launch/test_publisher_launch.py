#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
from tracetools_launch.action import Trace


def generate_launch_description():

    args = [
        DeclareLaunchArgument("name", default_value="test_publisher", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("test_publisher"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        DeclareLaunchArgument("trace", default_value="true", description="enable tracing"),
        DeclareLaunchArgument("trace_path", default_value=os.path.join(os.getcwd(), "traces"), description="path to save trace files"),
        DeclareLaunchArgument("input_topic", default_value="input_topic", description="topic to subscribe to"),
        DeclareLaunchArgument("input_topic2", default_value="input_topic2", description="second topic to subscribe to"),
        DeclareLaunchArgument("output_topic", default_value="~/output", description="topic to publish to"),
    ]

    nodes = [
        Node(
            package="test_publisher",
            executable="test_publisher",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[
                ("~/input", LaunchConfiguration("input_topic")),
                ("~/input2", LaunchConfiguration("input_topic2")),
                ("~/output", LaunchConfiguration("output_topic")),
            ],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="test_publisher",
            executable="dummy_publisher",
            namespace=LaunchConfiguration("namespace"),
            name="dummy_publisher",
            remappings=[
                ("~/topic", LaunchConfiguration("input_topic")),
                ("~/topic2", LaunchConfiguration("input_topic2")),
            ],
            output="screen",
            emulate_tty=True,
        ),
        Trace(
            session_name='trace',
            append_timestamp=True,
            base_path=LaunchConfiguration("trace_path"),
            condition=IfCondition(LaunchConfiguration("trace")),
        ),
    ]

    return LaunchDescription([
        *args,
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        *nodes,
    ])
