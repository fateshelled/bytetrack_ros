# bytetrack_ros

This package is a port of [ByteTrack](https://github.com/ifzhang/ByteTrack) to ROS2.
The object detector is assumed to be [YOLOX-ROS](https://github.com/Ar-Ray-code/YOLOX-ROS).


## Requirements
- ROS2 Foxy
- OpenCV
- Eigen3
- [bbox_ex_msgs](https://github.com/Ar-Ray-code/bbox_ex_msgs)
- [YOLOX-ROS](https://github.com/Ar-Ray-code/YOLOX-ROS)
- OpenVINO or TensorRT
- [ros_video_player](https://github.com/fateshelled/ros_video_player)

## How to Use (OpenVINO)
### Install
```
source /opt/ros/foxy/setup.bash
source /opt/intel/openvino_2021/bin/setupvars.sh

cd ~/ros2_ws/src
git clone https://github.com/fateshelled/bytetrack_ros
git clone https://github.com/fateshelled/ros_video_player
git clone --recursive https://github.com/Ar-Ray-code/YOLOX-ROS

# Download onnx file and Convert to IR format.
./YOLOX-ROS/weights/openvino/install.bash yolox_s
```

### DEMO
```
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

# launch with WebCam
ros2 launch bytetrack_cpp_node bytetrack_openvino.launch.py

# launch with Video File
ros2 launch bytetrack_cpp_node bytetrack_openvino.launch.py video_path:={video file path}

```

