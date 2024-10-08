# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node




def generate_launch_description():
    # Import the model urdf (load from file, xacro ...)
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'use_sim_time': True,'use_ros2_control': True, 'robot_description': robot_description_config.toxml(),  'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'left_wheel': 'left_wheel_joint',
                'right_wheel': 'right_wheel_joint',}
    robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher',
           output='screen',
           parameters=[params],
           arguments=['joint_state_broadcaster'])

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    params = {'use_sim_time': True}
    gazebo = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
       launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # RViz
    rviz_config_path = os.path.join(pkg_path, 'config', 'view_bot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
                   ],
        parameters=[{'qos_overrides./my_custom_model.subscriber.reliability': 'best effort'}],
        #             'qos_overrides./model/vehicle_green.subscriber.reliability': 'reliable'}],
        output='screen'
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_gui': True}, {'use_sim_time': True}],
    )

    # Spawn
    spawn = Node(package='ros_gz_sim', executable='create',
                 parameters=[{
                    'name': 'my_custom_model',
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'topic': '/robot_description'}],
                 output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )
        
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )
     
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        # rviz,
        joint_state_publisher_gui,
        spawn,
        bridge,
        diff_drive_spawner,
        joint_broad_spawner
    ])
