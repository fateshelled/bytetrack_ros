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
            "model_type",
            default_value="openvino",
            description="yolox model type. openvino or tensorrt"
        ),
        DeclareLaunchArgument(
            "device",
            default_value="CPU",
            description="model device. if openvino, select CPU, GPU, etc... if tensorrt, input GPU index."
        ),
        DeclareLaunchArgument(
            "image_size/height",
            default_value="640",
            description="model input image height."
        ),
        DeclareLaunchArgument(
            "image_size/width",
            default_value="640",
            description="model input image width."
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
                            "model_type": LaunchConfiguration("model_type"),
                            "device": LaunchConfiguration("device"),
                            "image_size/height": LaunchConfiguration("image_size/height"),
                            "image_size/width": LaunchConfiguration("image_size/width"),
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
                            "queue_size": 5,
                            "exact_sync": False,
                            "sub_image_topic_name": "/image_raw",
                            "sub_bboxes_topic_name": "/bytetrack/bounding_boxes",
                        }],
                        ),
                ],
                output='screen',
        )

    rqt_graph = launch_ros.actions.Node(
        package="rqt_graph", executable="rqt_graph",
    )

    return launch.LaunchDescription(
        launch_args + 
        [container] + 
        [rqt_graph]
    )