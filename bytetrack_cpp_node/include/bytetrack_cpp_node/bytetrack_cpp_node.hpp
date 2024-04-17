#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "bboxes_ex_msgs/msg/bounding_box.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"
#include "bytetrack_cpp/BYTETracker.h"

namespace bytetrack_cpp_node
{
using namespace bytetrack_cpp;

class ByteTrackNode : public rclcpp::Node
{
public:
    ByteTrackNode(const std::string& node_name, const rclcpp::NodeOptions& options);
    ByteTrackNode(const rclcpp::NodeOptions& options);

private:
    void initializeParameter_();
    void topic_callback_(const bboxes_ex_msgs::msg::BoundingBoxes::ConstSharedPtr msg);

    int video_fps_ = 30;
    int track_buffer_ = 30;
    std::string sub_bboxes_topic_name_;
    std::string pub_bboxes_topic_name_;
    std::unique_ptr<BYTETracker> tracker_;
    double track_thresh_;
    double high_thresh_;
    double match_thresh_;

    rclcpp::Publisher<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr pub_bboxes_;
    rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr sub_bboxes_;
};
} // namespace bytetrack_cpp_node
