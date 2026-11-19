import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node, SetParameter

def generate_launch_description():

    # ──────────────────────────────────────────────────────────────
    # Paths
    # ──────────────────────────────────────────────────────────────
    pkg_path = get_package_share_directory('URDFF_gz')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_description.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'classroom_world.sdf')
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    
    # ──────────────────────────────────────────────────────────────
    # Process xacro → URDF string
    # ──────────────────────────────────────────────────────────────    
    robot_desc = subprocess.check_output(
        ['xacro', xacro_file]
    ).decode('utf-8')

    # For Gazebo: use file:// absolute paths so Ignition can resolve meshes
    # regardless of IGN_GAZEBO_RESOURCE_PATH. Handles both package:// and
    # model:// URIs that may appear after xacro processing.
    robot_desc_gazebo = robot_desc.replace(
        'package://URDFF_gz', 'file://' + pkg_path
    ).replace(
        'model://URDFF_gz', 'file://' + pkg_path
    )

    tmp_urdf = '/tmp/robot_gazebo.urdf'
    with open(tmp_urdf, 'w') as f:
        f.write(robot_desc_gazebo)
        

    # pkg_path is e.g. /home/iheb/ros2_ws/install/URDFF_gz/share/URDFF_gz
    # IGN_GAZEBO_RESOURCE_PATH needs the PARENT of the package folder so that
    # model://URDFF_gz/meshes/... resolves correctly.
    pkg_parent = os.path.dirname(pkg_path)  # .../share
    
 
    # ──────────────────────────────────────────────────────────────
    # Start Gazebo with the world
    # ──────────────────────────────────────────────────────────────    
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r'],
        output='screen',
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'
        }
    )

    # ========================================
    # ROBOT STATE PUBLISHER
    # ========================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )
    # ========================================
    # Spawn the robot using ros_gz_sim
    # ========================================    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-file', tmp_urdf,
            '-z', '0.1'
        ],
        output='screen'
    )

    # ========================================
    # BRIDGES ROS ↔ GAZEBO
    # ========================================
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/pose@geometry_msgs/msg/PoseStamped@ignition.msgs.Pose',
            '/twist@geometry_msgs/msg/TwistStamped@ignition.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': True,
        }],
        output='screen'
    )
    
    # ──────────────────────────────────────────────────────────────
    # EKF
    # ──────────────────────────────────────────────────────────────    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path,
        {'use_sim_time': True}
        ]
    )
    
    # ──────────────────────────────────────────────────────────────
    # Launch sequence
    # ──────────────────────────────────────────────────────────────
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        gazebo,
        robot_state_publisher,
        bridge,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=7.0, actions=[ekf_node]),
    ])
