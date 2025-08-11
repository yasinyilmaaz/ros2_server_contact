from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1) TurtleBot3 Gazebo simülasyonu
    turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_maze_world.launch.py'
            ])
        )
    )

    # 2) Cartographer (SLAM)
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_cartographer'),
                'launch',
                'cartographer.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # 3) Bizim modüler nodeler
    map_to_jpeg = Node(
        package='data_contact_py_pkg',
        executable='map_to_jpeg_node',
        name='map_to_jpeg_node',
        parameters=[{
            'image_width': 480,
            'image_height': 480,
            'jpeg_quality': 85,
            'draw_robot_pose': True,
            'map_topic': '/map',
            'odom_topic': '/odom',
            'output_topic': '/map_jpeg_b64',
        }]
    )

    speed_estimator = Node(
        package='data_contact_py_pkg',
        executable='speed_estimator_node',
        name='speed_estimator_node',
        parameters=[{
            'odom_topic': '/odom',
            'use_twist_if_available': True,
            'out_v_topic': '/speed_linear',
            'out_w_topic': '/speed_angular',
        }]
    )

    obstacle_detector = Node(
        package='data_contact_py_pkg',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        parameters=[{
            'scan_topic': '/scan',
            'range_topic': '/range',
            'range_threshold': 0.6,
            'out_flag_topic': '/obstacle_detected',
            'out_dist_topic': '/obstacle_distance',
        }]
    )

    battery_passthrough = Node(
        package='data_contact_py_pkg',
        executable='battery_passthrough_node',
        name='battery_passthrough_node',
        parameters=[{
            'input_topic': '/battery_status',
            'output_topic': '/battery_percent',
        }]
    )

    server_bridge = Node(
        package='data_contact_py_pkg',
        executable='server_bridge_node',
        name='server_bridge_node',
        parameters=[{
            'server_url': 'http://localhost:8080/telemetry',
            'publish_rate_hz': 5.0,
            'robot_id': 'robot_001',
            'map_topic': '/map_jpeg_b64',
            'v_topic': '/speed_linear',
            'w_topic': '/speed_angular',
            'obs_flag_topic': '/obstacle_detected',
            'obs_dist_topic': '/obstacle_distance',
            'battery_topic': '/battery_percent',
        }]
    )

    return LaunchDescription([
        turtlebot3_world,

        TimerAction(
            period=5.0,
            actions=[cartographer]
        ),

        TimerAction(
            period=10.0,
            actions=[
                map_to_jpeg,
                speed_estimator,
                obstacle_detector,
                battery_passthrough,
                server_bridge,
            ]
        ),
    ])
