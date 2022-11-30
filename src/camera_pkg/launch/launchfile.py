#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    camera_node = Node(
        package="camera_pkg",
        executable="camera_node"
    )
    image_processing_node = Node(
        package="camera_pkg",
        executable="image_processing_node"
    )
    gui_node = Node(
        package="camera_pkg",
        executable="gui_node"
    )

    ld.add_action(camera_node)
    ld.add_action(image_processing_node)
    #ld.add_action(gui_node)
    return ld
