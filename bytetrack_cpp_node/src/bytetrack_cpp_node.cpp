#include "bytetrack_cpp_node/bytetrack_cpp_node.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>

#include "bytetrack_cpp_node/coco_names.hpp"

namespace bytetrack_cpp_node
{
using namespace bytetrack_cpp;

std::vector<Object> BoundingBoxes2Objects(
    const std::vector<bboxes_ex_msgs::msg::BoundingBox> &bboxes)
{
    std::vector<Object> objects;
    objects.reserve(bboxes.size());

    const float scale = 1.0f;
    for (const auto &bbox : bboxes)
    {
        Object obj;
        obj.rect.x = (bbox.xmin) / scale;
        obj.rect.y = (bbox.ymin) / scale;
        obj.rect.width = (bbox.xmax - bbox.xmin) / scale;
        obj.rect.height = (bbox.ymax - bbox.ymin) / scale;

        const auto it = std::find(COCO_CLASSES, COCO_CLASSES + 80, bbox.class_id);
        if (it != COCO_CLASSES + 80)
        {
            obj.label = std::distance(COCO_CLASSES, it);
        }

        obj.prob = bbox.probability;
        objects.push_back(obj);
    }
    return objects;
}
std::vector<bboxes_ex_msgs::msg::BoundingBox> STrack2BoundingBoxes(
    const std::vector<STrack> &trackers)
{
    std::vector<bboxes_ex_msgs::msg::BoundingBox> bboxes;
    bboxes.reserve(trackers.size());

    for (const auto &tracker : trackers)
    {
        bboxes_ex_msgs::msg::BoundingBox bbox;
        bbox.ymin = tracker.tlbr[0];
        bbox.xmin = tracker.tlbr[1];
        bbox.ymax = tracker.tlbr[2];
        bbox.xmax = tracker.tlbr[3];
        bbox.id = tracker.track_id;
        bbox.class_id = COCO_CLASSES[tracker.label];
        // bbox.center_dist = 0.0;
        // bbox.img_height = 0;
        // bbox.img_width = 0;
        bboxes.push_back(bbox);
    }
    return bboxes;
}

ByteTrackNode::ByteTrackNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp::Node("bytetrack_cpp_node", node_name, options)
{
}

ByteTrackNode::ByteTrackNode(const rclcpp::NodeOptions &options) : ByteTrackNode("", options)
{
    this->initializeParameter_();
    this->tracker_ =
        std::make_unique<BYTETracker>(this->video_fps_, this->track_buffer_, this->track_thresh_,
                                      this->high_thresh_, this->match_thresh_);

    this->sub_bboxes_ = this->create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>(
        this->sub_bboxes_topic_name_, 10,
        std::bind(&ByteTrackNode::topic_callback_, this, std::placeholders::_1));
    this->pub_bboxes_ = this->create_publisher<bboxes_ex_msgs::msg::BoundingBoxes>(
        this->pub_bboxes_topic_name_, 10);
}
void ByteTrackNode::initializeParameter_()
{
    this->video_fps_ = this->declare_parameter<int>("video_fps", 30);
    this->track_buffer_ = this->declare_parameter<int>("track_buffer", 30);
    this->track_thresh_ = this->declare_parameter<double>("track_thresh", 0.5);
    this->high_thresh_ = this->declare_parameter<double>("high_thresh", 0.6);
    this->match_thresh_ = this->declare_parameter<double>("match_thresh", 0.8);
    this->sub_bboxes_topic_name_ =
        this->declare_parameter<std::string>("sub_bboxes_topic_name", "yolox/bounding_boxes");
    this->pub_bboxes_topic_name_ =
        this->declare_parameter<std::string>("pub_bboxes_topic_name", "bytetrack/bounding_boxes");
}
void ByteTrackNode::topic_callback_(const bboxes_ex_msgs::msg::BoundingBoxes::ConstSharedPtr msg)
{
    bboxes_ex_msgs::msg::BoundingBoxes bboxes;
    bboxes.header = msg->header;
    bboxes.image_header = msg->image_header;

    const vector<Object> objects = BoundingBoxes2Objects(msg->bounding_boxes);
    const vector<STrack> output_stracks = this->tracker_->update(objects);
    RCLCPP_INFO(this->get_logger(), "Detect objects: %ld, Output Tracker: %ld", objects.size(),
                output_stracks.size());
    bboxes.bounding_boxes = STrack2BoundingBoxes(output_stracks);
    this->pub_bboxes_->publish(bboxes);
}
} // namespace bytetrack_cpp_node

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    rclcpp::spin(std::make_shared<bytetrack_cpp_node::ByteTrackNode>(node_options));
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bytetrack_cpp_node::ByteTrackNode)
