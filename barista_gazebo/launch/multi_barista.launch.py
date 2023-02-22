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


def gen_robot_list(num_r):

    robots = []
    pose_1 = [7.709494, -9.752420]
    pose_2 = [8.590692, -8.685888]
    pose_3 = [9.556409, -7.642379]

    pose_array = [pose_1, pose_2, pose_3]
    robot_name_array = ["barista_1", "barista_2", "mule_1"]

    number_of_robots = min(num_r, len(robot_name_array))

    for i in range(number_of_robots):
        print("########################################### = "+str(i))
        print(str(robot_name_array))
        robot_name = robot_name_array[i]
        x_pos = pose_array[i][0]
        y_pos = pose_array[i][1]
        robots.append({'name': robot_name, 'x_pose': x_pos,
                      'y_pose': y_pos, 'z_pose': 0.1, 'Y_pose': 0.0})

    print("############### ROBOTS MULTI ARRAY="+str(robots))

    return robots


def launch_setup(context, *args, **kwargs):

    number_of_robots = int(LaunchConfiguration(
        'number_of_robots').perform(context))


    launch_file_dir = os.path.join(get_package_share_directory(
        'barista_description'), 'launch')

    # Names and poses of the robots
    robots = gen_robot_list(number_of_robots)

    # We create the list of spawn robots commands
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    launch_file_dir, 'spawn.launch.py')),
                launch_arguments={
                    'x_spawn': TextSubstitution(text=str(robot['x_pose'])),
                    'y_spawn': TextSubstitution(text=str(robot['y_pose'])),
                    'entity_name': robot['name']
                }.items()))

    # Create the launch description and populate
    ld = LaunchDescription()

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return [ld]


def generate_launch_description():

    number_of_robots_arg = DeclareLaunchArgument(
        'number_of_robots', default_value='1')

    return LaunchDescription([
        number_of_robots_arg,
        OpaqueFunction(function=launch_setup)
    ])
