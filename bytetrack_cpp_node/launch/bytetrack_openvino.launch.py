import os
import sys
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "video_path",
            default_value="/dev/video0",
            description="input video file path."
        ),
        DeclareLaunchArgument(
            "video_fps",
            default_value="30",
            description="video fps."
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value="./install/yolox_ros_cpp/share/yolox_ros_cpp/weights/openvino/yolox_s.xml",
            description="yolox model path."
        ),
        DeclareLaunchArgument(
            "model_version",
            default_value="0.1.1rc0",
            description="yolox model version."
        ),
        DeclareLaunchArgument(
            "device",
            default_value="CPU",
            description="model device. CPU, GPU, MYRIAD, etc..."
        ),
        DeclareLaunchArgument(
            "conf",
            default_value="0.1",
            description="yolox confidence threshold."
        ),
        DeclareLaunchArgument(
            "nms",
            default_value="0.7",
            description="yolox nms threshold"
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
    container = ComposableNodeContainer(
                name='bytetrack_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='ros_video_player',
                        plugin='ros_video_player::VideoPlayerNode',
                        name='ros_video_player',
                        parameters=[{
                            "publish_topic_name": "/image_raw",
                            "video_path": LaunchConfiguration("video_path"),
                            "frame_id": "map",
                            "loop": False,
                            "speed": 1.0
                        }]),
                    ComposableNode(
                        package='yolox_ros_cpp',
                        plugin='yolox_ros_cpp::YoloXNode',
                        name='yolox_ros_cpp',
                        parameters=[{
                            "model_path": LaunchConfiguration("model_path"),
                            "model_type": "openvino",
                            "model_version": LaunchConfiguration("model_version"),
                            "device": LaunchConfiguration("device"),
                            "conf": LaunchConfiguration("conf"),
                            "nms": LaunchConfiguration("nms"),
                            "imshow_isshow": False,
                            "src_image_topic_name": "/image_raw",
                            "publish_image_topic_name": "/yolox/image_raw",
                            "publish_boundingbox_topic_name": "/yolox/bounding_boxes",
                        }],
                        ),
                    ComposableNode(
                        package='bytetrack_cpp_node',
                        plugin='bytetrack_cpp_node::ByteTrackNode',
                        name='bytetrack_cpp_node',
                        parameters=[{
                            "video_fps": LaunchConfiguration("video_fps"),
                            "track_buffer": 30,
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
                ],
                output='screen',
        )

    # rqt_graph = launch_ros.actions.Node(
    #     package="rqt_graph", executable="rqt_graph",
    # )

    return launch.LaunchDescription(
        launch_args
        + [container]
        # + [rqt_graph]
    )
