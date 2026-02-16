import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_desc_dir = get_package_share_directory('robot_description')
    
    map_pub_dir = get_package_share_directory('map_publisher')
    
    default_model_path = os.path.join(robot_desc_dir, 'src', 'description', 'robot.urdf')
    default_rviz_config_path = os.path.join(robot_desc_dir, 'rviz', 'config.rviz')
    default_map_path = os.path.join(map_pub_dir, 'maps', 'my_map.yaml')

    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()
        
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc}
        ])
    
    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    # Map Publisher
    map_publisher_node = Node(
        package='map_publisher',
        executable='map_publisher_node',
        name='map_publisher_node',
        output='screen',
        parameters=[
            {'yaml_file': LaunchConfiguration('map_yaml')}
        ]
    )
    
    # A* Planner
    astar_planner_node = Node(
        package='map_publisher',
        executable='astar_planner_node',
        name='astar_planner_node',
        output='screen'
    )
    
    # Static TF: map -> odom
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot model file'
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        DeclareLaunchArgument(
            name='map_yaml',
            default_value=default_map_path,
            description='Path to map yaml file'
        ),
        
        # Nodes
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        map_publisher_node,
        astar_planner_node,
        static_tf_map_odom,
        static_tf_odom_base,
    ])
