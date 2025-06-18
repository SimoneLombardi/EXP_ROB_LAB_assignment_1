"""
Spawn Robot Description
"""
import os
import numpy as np
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit

# robot position
rb_x = 0.0
rb_y = 0.0


mk1 = 11
mkn = 40
mk_num = 8
x_sh=-0.86428
y_sh=-0.101106
    
def compute_marker_pos(mk_pos, mk_yaw):
    # Rotation matrix around Z axis
    Rz = np.array([
        [np.cos(mk_yaw), -np.sin(mk_yaw)],
        [np.sin(mk_yaw),  np.cos(mk_yaw)]
        
    ])
    # mk_pos is expected as (x, y), so use those values
    mk_pos_vec = np.array([[mk_pos[0]], [mk_pos[1]]])
    mk_frame = np.array([[-x_sh], [-y_sh]])
    
    mk_frame_base = np.matmul(Rz, mk_frame)
    mk_FR_pos = mk_pos_vec + mk_frame_base
    
    return mk_FR_pos[0, 0], mk_FR_pos[1, 0]
    


def generate_launch_description():
    test_robot_description_share = FindPackageShare(package='erl_ass1_pkg').find('erl_ass1_pkg')
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot4.xacro')
    rviz_config_path = os.path.join(test_robot_description_share, 'config/rviz.rviz')
    
    
    models_dir_path = os.path.expanduser('~/.gazebo/models')
    #sample of marker models
    marker_id_list = list(range(mk1, mkn+1))
    marker_list = random.sample(marker_id_list, mk_num)
    str(marker_list) # convert the id to string
    #marker model position
    mk_pos = [(1.0,0.0),(.71,0.71),(0.0,1.0),(-0.71,0.71),(-1.0,0.0),(-0.71,-0.71),(0.0,-1.0),(0.71,-0.71)]
    mk_yaw = [np.pi, np.pi*5/4, np.pi*3/2, -np.pi/4, 0, np.pi/4, np.pi/2, np.pi*3/4]
    
    # lista dei cmd per spawnare i marker      
    mk_spawn = list(())
    
    for i in range(mk_num):
        mk_pos_corr = compute_marker_pos(mk_pos[i], mk_yaw[i])
        mk_spawn.append(
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', f'marker_{marker_list[i]}', 
                     '-file', os.path.join(models_dir_path, f'marker{marker_list[i]}_box/model.sdf'),
                     '-x', str(mk_pos_corr[0]), 
                     '-y', str(mk_pos_corr[1]), 
                     '-z', '0.0',
                     '-Y', str(mk_yaw[i])],
                output='screen'
            )
        )
        
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    node_robot_manager = Node(
        package="erl_ass1_pkg",
        executable="robot_manager",
        name="robot_manager",
    )
    
    # add the controller nodes
    node_camera_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["controller_camera_joint"]
    )
    
    node_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )
    
    mk_id_publisher = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
    )
    
    shutdown_on_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_robot_manager,
            on_exit=[Shutdown(reason='Main node exited')]
        )
    )
   

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'my_test_robot', '-topic', '/robot_description', '-x', str(rb_x), '-y', str(rb_y)],
                        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        ExecuteProcess(
            cmd=['gazebo', '--verbose','worlds/empty.world', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        node_robot_state_publisher,
        node_joint_state_publisher,
        spawn_entity,

        node_camera_controller,
        node_joint_state_broadcaster,
        node_robot_manager,
        
        mk_spawn[0], mk_spawn[1], mk_spawn[2], mk_spawn[3], 
        mk_spawn[4], mk_spawn[5], mk_spawn[6], mk_spawn[7],
        
        mk_id_publisher,
        shutdown_on_exit,
        
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'),
    ])
