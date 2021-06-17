# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Ignition Gazebo with command line arguments."""

import os

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import parse_package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')])}

    return LaunchDescription([
        DeclareLaunchArgument(
            'ign_args', default_value='',
            description='Arguments to be passed to Ignition Gazebo'),
        ExecuteProcess(
            cmd=['ign gazebo',
                 OpaqueFunction(__get_ign_args)
                 ],
            output='screen',
            additional_env=env,
            shell=True
        )
    ])


def __get_ign_args(context: LaunchContext):
    ign_args = LaunchConfiguration('ign_args').perform(context)
    if '--force_version' not in ign_args:
        ign_args += f' --force_version {IGN_GAZEBO_DEFAULT_VERSION}'

    return ign_args


def __get_ign_gazebo_default_version():
    manifest_path = os.path.join(*[get_package_share_directory('ros_ign_gazebo'), 'package.xml'])

    # this will fail if workspace was built with '--merge-install'
    this_pkg = parse_package(manifest_path)

    this_pkg.evaluate_conditions(os.environ)
    lookup_name = next(dep.name for dep in this_pkg.exec_depends
                       if dep.evaluated_condition
                       if 'ignition-gazebo' in dep.name)

    return lookup_name.replace('ignition-gazebo', '')


IGN_GAZEBO_DEFAULT_VERSION = __get_ign_gazebo_default_version()
