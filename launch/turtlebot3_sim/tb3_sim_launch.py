# Copyright (c) 2018 Intel Corporation
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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import launch.events
import lifecycle_msgs.msg

def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pilot_bringup_dir = get_package_share_directory('navigation_experiments_mc_bts_pddl')
    launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to execute gzclient)')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        #                            'worlds/turtlebot3_worlds/waffle.model'),
        default_value=os.path.join(pilot_bringup_dir, 'worlds', 'waffle_house.model'),
        description='Full path to world model file to load')

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', world],
        cwd=[launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    urdf = os.path.join(nav2_bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[urdf])
    

    pcl2laser_cmd = LifecycleNode(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_managed',
        name='pointcloud_to_laser',
        remappings=[('cloud_in', '/intel_realsense_r200_depth/points'),
                    ('scan', '/mros_scan')],
        parameters=[{
            'target_frame': 'base_scan',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -1.5708,  # -M_PI/2
            'angle_max': 1.5708,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 4.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
    )
    emit_event_to_request_that_pcl2laser_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(pcl2laser_cmd),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    laser_resender_cmd = LifecycleNode(
        name='laser_resender',
        package='laser_resender',
        executable='laser_resender_node',
        output='screen')

    battery_contingency_cmd = Node(
        name='battery_contingency_sim',
        package='mros_contingencies_sim',
        executable='battery_contingency_sim_node',
        output='screen')
    
    components_file_path = (get_package_share_directory('mros_modes_observer') +
        '/params/components.yaml')

    modes_observer_node = Node(
        package='mros_modes_observer',
        executable='modes_observer_node',
        parameters=[{'componentsfile': components_file_path}],
        output='screen')
    
    emit_event_to_request_that_laser_resender_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(laser_resender_cmd),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    shm_model_path = (get_package_share_directory('navigation_experiments_mc_bts_pddl') +
        '/params/pilot_modes.yaml')

    # Start as a normal node is currently not possible.
    # Path to SHM file should be passed as a ROS parameter.
    mode_manager_node = Node(
        package='system_modes',
        executable='mode_manager',
        parameters=[{'modelfile': shm_model_path}],
        output='screen')

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    # Add system modes manager
    ld.add_action(mode_manager_node)
    
    # Add system modes observer node
    ld.add_action(modes_observer_node)

    ld.add_action(pcl2laser_cmd)
    ld.add_action(laser_resender_cmd)
    ld.add_action(battery_contingency_cmd)
    ld.add_action(emit_event_to_request_that_pcl2laser_configure_transition)
    ld.add_action(emit_event_to_request_that_laser_resender_configure_transition)

    return ld
