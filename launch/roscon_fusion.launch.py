from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
import subprocess

def copy_zed_config():
    zedx_config = FindPackageShare('miivii_image_projection_based_fusion').find('miivii_image_projection_based_fusion') + '/config/zedx.yaml'
    zedx_target = FindPackageShare('zed_wrapper').find('zed_wrapper') + '/config/zedx.yaml'

    print(zedx_config)
    print(zedx_target)

    subprocess.run(['rm', '-f', zedx_target])
    subprocess.run(['cp', '-f', zedx_config, zedx_target])

def generate_launch_description():
    copy_zed_config()

    # 定义参数
    data_path = LaunchConfiguration('data_path')
    pointcloud_topic = LaunchConfiguration('input_pointcloud')
    rviz_arg = LaunchConfiguration('rviz')
    use_decompress = LaunchConfiguration('use_decompress')
    camera_topic = LaunchConfiguration('camera_topic')
    camera_info = LaunchConfiguration('camera_info')
    interface = LaunchConfiguration('interface')
    receiver_interval_sec = LaunchConfiguration('receiver_interval_sec')
    input_frame = LaunchConfiguration('input_frame')
    output_objects = LaunchConfiguration('output_objects')
    output_scan = LaunchConfiguration('output_scan')
    publish_radar_track = LaunchConfiguration('publish_radar_track')
    publish_radar_scan = LaunchConfiguration('publish_radar_scan')
    output_frame = LaunchConfiguration('output_frame')
    sequential_publish = LaunchConfiguration('sequential_publish')
    size_x = LaunchConfiguration('size_x')
    size_y = LaunchConfiguration('size_y')
    pic_path = LaunchConfiguration('pic_path')
    camera_calibration_file = LaunchConfiguration('camera_calibration_file')


    declare_data_path_cmd = DeclareLaunchArgument(
        'data_path', default_value=os.path.join(os.getenv('HOME'), 'autoware_data'),
        description='Packages data and artifacts directory path'
    )
    declare_pointcloud_topic_cmd = DeclareLaunchArgument(
        'input_pointcloud', default_value='/lidar_points',
        description='Input pointcloud topic name'
    )
    declare_rviz_arg_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to run rviz'
    )
    declare_use_decompress_cmd = DeclareLaunchArgument(
        'use_decompress', default_value='false',
        description='Whether to use decompression'
    )
    declare_camera_topic_cmd = DeclareLaunchArgument(
        'camera_topic', default_value='/miivii_gmsl/image2',
        description='Camera topic name'
    )
    declare_camera_info_cmd = DeclareLaunchArgument(
        'camera_info', default_value='/miivii_gmsl/camera_info2',
        description='Camera info topic name'
    )
    declare_interface_cmd = DeclareLaunchArgument(
        'interface', default_value='can0',
        description='Interface name'
    )
    declare_receiver_interval_sec_cmd = DeclareLaunchArgument(
        'receiver_interval_sec', default_value='0.2',
        description='Receiver interval in seconds'
    )
    declare_input_frame_cmd = DeclareLaunchArgument(
        'input_frame', default_value='/from_can_bus',
        description='Input frame name'
    )
    declare_output_objects_cmd = DeclareLaunchArgument(
        'output_objects', default_value='objects_raw',
        description='Output objects topic name'
    )
    declare_output_scan_cmd = DeclareLaunchArgument(
        'output_scan', default_value='scan',
        description='Output scan topic name'
    )
    declare_publish_radar_track_cmd = DeclareLaunchArgument(
        'publish_radar_track', default_value='true',
        description='Whether to publish radar track'
    )
    declare_publish_radar_scan_cmd = DeclareLaunchArgument(
        'publish_radar_scan', default_value='false',
        description='Whether to publish radar scan'
    )
    declare_output_frame_cmd = DeclareLaunchArgument(
        'output_frame', default_value='ars408',
        description='Output frame name'
    )
    declare_sequential_publish_cmd = DeclareLaunchArgument(
        'sequential_publish', default_value='false',
        description='Whether to publish sequentially'
    )
    declare_size_x_cmd = DeclareLaunchArgument(
        'size_x', default_value='1.8',
        description='Size x dimension'
    )
    declare_size_y_cmd = DeclareLaunchArgument(
        'size_y', default_value='1.8',
        description='Size y dimension'
    )
    declare_pic_path_cmd = DeclareLaunchArgument(
        'pic_path', default_value=FindPackageShare('autoware_launch').find('autoware_launch') + '/rviz/image/autoware.png',
        description='Path to the picture file'
    )

    declare_camera_calibration_file_cmd = DeclareLaunchArgument(
        'camera_calibration_file', default_value=FindPackageShare('miivii_image_projection_based_fusion').find('miivii_image_projection_based_fusion') + '/config/calibration/tieriv_c2.yaml',
    )

     # 获取robot_description
    model_file = FindPackageShare('miivii_image_projection_based_fusion').find('miivii_image_projection_based_fusion') + '/config/urdf/sensors.xacro'
    config_dir = FindPackageShare('miivii_image_projection_based_fusion').find('miivii_image_projection_based_fusion') + '/config/calibration'
    robot_description = subprocess.check_output(['xacro', model_file, f'config_dir:={config_dir}']).decode('utf-8')

    # 节点定义
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }],
        output='screen'
    )

    zedx_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('zed_wrapper'), '/launch/zed_camera.launch.py']),
        launch_arguments={
            'camera_model': 'zedx'
        }.items()
    )

    hesai_lidar_node = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='screen'
    )

    miivii_gmsl_camera_node = Node(
        package='miivii_gmsl_camera',
        executable='miivii_gmsl_camera_node',
        output='screen',
        parameters=[
            {'sync_freq': 15},
            {'video2.active': True},
            {'video2.node_name': '/dev/video2'},
            {'video2.camera_res': '2880x1860'},
            {'video2.output_res': '2880x1860'},
            {'video2.is_tiervi_camera': True},
            {'video2.params_file': camera_calibration_file},
            {'video3.active': True},
            {'video3.node_name': '/dev/video3'},
            {'video3.camera_res': '2880x1860'},
            {'video3.output_res': '1280x720'},
            {'video3.is_tiervi_camera': True},
            {'video4.active': True},
            {'video4.node_name': '/dev/video4'},
            {'video4.camera_res': '2880x1860'},
            {'video4.output_res': '1280x720'},
            {'video4.is_tiervi_camera': True},
            {'video5.active': True},
            {'video5.node_name': '/dev/video5'},
            {'video5.camera_res': '2880x1860'},
            {'video5.output_res': '1280x720'},
            {'video5.is_tiervi_camera': True},
        ]
    )

    can_socket_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros2_socketcan'), '/launch/socket_can_receiver.launch.py']),
        launch_arguments={
            'interface': interface,
            'interval_sec': receiver_interval_sec
        }.items()
    )

    radar_node = Node(
        package='pe_ars408_ros',
        executable='pe_ars408_node',
        name='pe_ars408_node',
        output='screen',
        parameters=[
            {'publish_radar_track': publish_radar_track},
            {'publish_radar_scan': publish_radar_scan},
            {'output_frame': output_frame},
            {'sequential_publish': sequential_publish},
            {'size_x': size_x},
            {'size_y': size_y}
        ],
        remappings=[
            ('~/input/frame', input_frame),
            ('~/output/objects', output_objects),
            ('~/output/scan', output_scan)
        ]
    )


    image_publisher_node = Node(
        package='autoware_misc',
        executable='miivii_image_publisher_node',
        output='screen',
        parameters=[{'image_path': pic_path}]
    )

    project_points_node = Node(
        package='miivii_image_projection_based_fusion',
        executable='project_points_to_image_node',
        output='screen',
        parameters=[
            {'camera_num': 1},
            {'pointcloud_buffer_size': 5},
            {'input_pointcloud_topic': pointcloud_topic},
            {'input/image0': camera_topic},
            {'input/camera_info0': camera_info},
            {'output_image_topic_prefix': '/miivii_projection/image'},
            {'output_pointcloud_topic': '/test_points'}
        ]
    )

    miivii_status_monitor_node = Node(
        package="autoware_misc",
        executable="miivii_status_monitor_node",
        output='screen',
        parameters=[
            {'camerainfo_topics': ['/zed/zed_node/left/camera_info']},
            {'pointcloud_topic': "/lidar_points"},
            {'can_topic': "/objects_raw"}
        ]
    )

    actuation_message_converter_node = Node(
        package="autoware_misc",
        executable="actuation_message_converter_node_exe",
        output='screen'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', FindPackageShare('miivii_image_projection_based_fusion').find('miivii_image_projection_based_fusion') + '/config/rviz/fusion_new.rviz'],
    )

    # 定义延迟启动动作
    delayed_actions = TimerAction(period=25.0, actions=[
        image_publisher_node,
        hesai_lidar_node,
        miivii_gmsl_camera_node,
        can_socket_node,
        radar_node,
        project_points_node,
        rviz2_node
    ])

    delayed_actions_for_converter = TimerAction(period=35.0, actions=[
        miivii_status_monitor_node,
        actuation_message_converter_node
    ])
        

    # 创建launch描述
    ld = LaunchDescription([
        declare_data_path_cmd,
        declare_pointcloud_topic_cmd,
        declare_rviz_arg_cmd,
        declare_use_decompress_cmd,
        declare_camera_topic_cmd,
        declare_camera_info_cmd,
        declare_interface_cmd,
        declare_receiver_interval_sec_cmd,
        declare_input_frame_cmd,
        declare_output_objects_cmd,
        declare_output_scan_cmd,
        declare_publish_radar_track_cmd,
        declare_publish_radar_scan_cmd,
        declare_output_frame_cmd,
        declare_sequential_publish_cmd,
        declare_size_x_cmd,
        declare_size_y_cmd,
        declare_pic_path_cmd,
        declare_camera_calibration_file_cmd,

        static_transform_publisher_node,
        robot_state_publisher_node,
        zedx_camera_node,
        delayed_actions,
        delayed_actions_for_converter
    ])

    return ld
