#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import OpaqueFunction


def gen_robot_info():

    pose_3 = [9.556409, -7.642379]

    robot_name = "mule_1"
    x_pos = pose_3[0]
    y_pos = pose_3[1]
    robot = {'name': robot_name, 'x_pose': x_pos,
                    'y_pose': y_pos, 'z_pose': 0.1, 'Y_pose': 0.0}

    print("############### ROBOTS MULTI ARRAY="+str(robot))

    return robot


def launch_setup(context, *args, **kwargs):

    launch_file_dir = os.path.join(get_package_share_directory(
        'barista_description'), 'launch')

    # Names and poses of the robots
    robot = gen_robot_info()

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                launch_file_dir, 'spawn.launch.py')),
            launch_arguments={
                'x_spawn': TextSubstitution(text=str(robot['x_pose'])),
                'y_spawn': TextSubstitution(text=str(robot['y_pose'])),
                'entity_name': robot['name']
            }.items()))

    return [ld]


def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
