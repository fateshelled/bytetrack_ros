import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(
            'video_path',
            default_value='/dev/video0',
            description="input video file path."
        ),
        DeclareLaunchArgument(
            "video_fps",
            default_value="30",
            description="video fps."
        ),
        DeclareLaunchArgument(
            "track_thresh",
            default_value="0.5",
            description="track threshold."
        ),
        DeclareLaunchArgument(
            "high_thresh",
            default_value="0.6",
            description="track threshold."
        ),
        DeclareLaunchArgument(
            "match_thresh",
            default_value="0.8",
            description="track threshold."
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value="./src/YOLOX-ROS/weights/tensorrt/yolox_tiny.trt",
            description='yolox model path.'
        ),
        DeclareLaunchArgument(
            'p6',
            default_value='false',
            description='with p6.'
        ),
        DeclareLaunchArgument(
            'class_labels_path',
            default_value='',
            description='if use custom model, set class name labels. '
        ),
        DeclareLaunchArgument(
            'num_classes',
            default_value='80',
            description='num classes.'
        ),
        DeclareLaunchArgument(
            'model_version',
            default_value='0.1.1rc0',
            description='yolox model version.'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description='GPU index. Set in string type. ex 0'
        ),
        DeclareLaunchArgument(
            'conf',
            default_value='0.1',
            description='yolox confidence threshold.'
        ),
        DeclareLaunchArgument(
            'nms',
            default_value='0.7',
            description='yolox nms threshold'
        ),
        DeclareLaunchArgument(
            "save_video",
            default_value="true",
            description="record video."
        ),
        DeclareLaunchArgument(
            "save_video_name",
            default_value="output.mp4",
            description="record video filename."
        ),
        DeclareLaunchArgument(
            "save_video_fps",
            default_value="30",
            description="record video fps."
        ),
        DeclareLaunchArgument(
            "save_video_codec",
            default_value="MJPG",
            description="record video codec."
        ),
    ]
    composable_nodes = [
        # ComposableNode(
        #     package='v4l2_camera',
        #     plugin='v4l2_camera::V4L2Camera',
        #     name='v4l2_camera',
        #     parameters=[{
        #             'video_device': LaunchConfiguration('video_path'),
        #             'image_size': [640, 480]
        #     }]
        # ),
        ComposableNode(
            package='ros_video_player',
            plugin='ros_video_player::VideoPlayerNode',
            name='ros_video_player',
            parameters=[{
                "publish_topic_name": "/image_raw",
                "video_path": LaunchConfiguration("video_path"),
                "frame_id": "map",
                "loop": False,
                "speed": 1.0,
                "video_buffer_size": 1,
            }]),
        ComposableNode(
            package='yolox_ros_cpp',
            plugin='yolox_ros_cpp::YoloXNode',
            name='yolox_ros_cpp',
            parameters=[{
                    'model_path': LaunchConfiguration('model_path'),
                    'p6': LaunchConfiguration('p6'),
                    'class_labels_path': LaunchConfiguration('class_labels_path'),
                    'num_classes': LaunchConfiguration('num_classes'),
                    'model_type': 'tensorrt',
                    'model_version': LaunchConfiguration('model_version'),
                    'tensorrt/device': LaunchConfiguration('device'),
                    'conf': LaunchConfiguration('conf'),
                    'nms': LaunchConfiguration('nms'),
                    'imshow_isshow': False,
                    "src_image_topic_name": "/image_raw",
                    "publish_image_topic_name": "/yolox/image_raw",
                    'publish_boundingbox_topic_name': "/yolox/bounding_boxes",
            }],
        ),
        ComposableNode(
            package='bytetrack_cpp_node',
            plugin='bytetrack_cpp_node::ByteTrackNode',
            name='bytetrack_cpp_node',
            parameters=[{
                "video_fps": LaunchConfiguration("video_fps"),
                "track_buffer": 30,
                "track_thresh": LaunchConfiguration("track_thresh"),
                "high_thresh": LaunchConfiguration("high_thresh"),
                "match_thresh": LaunchConfiguration("match_thresh"),
                "sub_bboxes_topic_name": "/yolox/bounding_boxes",
                "pub_bboxes_topic_name": "/bytetrack/bounding_boxes",
            }],
        ),
        ComposableNode(
            package='bytetrack_viewer',
            plugin='bytetrack_viewer::ByteTrackViewer',
            name='bytetrack_viewer',
            parameters=[{
                "queue_size": 10,
                "exact_sync": False,
                "sub_image_topic_name": "/image_raw",
                "sub_bboxes_topic_name": "/bytetrack/bounding_boxes",
                "save_video": LaunchConfiguration("save_video"),
                "save_video_name": LaunchConfiguration("save_video_name"),
                "save_video_fps": LaunchConfiguration("save_video_fps"),
                "save_video_codec": LaunchConfiguration("save_video_codec"),
            }],
        ),
    ]

    container = ComposableNodeContainer(
        name='bytetrack_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    return launch.LaunchDescription(
        launch_args + [container]
    )
