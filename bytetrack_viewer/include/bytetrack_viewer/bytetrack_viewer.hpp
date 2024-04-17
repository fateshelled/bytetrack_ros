#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#define CV_BRIDGE_VERSION_GTE(major, minor, patch) \
    ((major < CV_BRIDGE_VERSION_MAJOR)   ? true    \
     : (major > CV_BRIDGE_VERSION_MAJOR) ? false   \
     : (minor < CV_BRIDGE_VERSION_MINOR) ? true    \
     : (minor > CV_BRIDGE_VERSION_MINOR) ? false   \
     : (patch < CV_BRIDGE_VERSION_PATCH) ? true    \
     : (patch > CV_BRIDGE_VERSION_PATCH) ? false   \
                                         : true)

#if CV_BRIDGE_VERSION_GTE(3, 4, 0)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/opencv.hpp>

#include "bboxes_ex_msgs/msg/bounding_box.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"

namespace bytetrack_viewer
{

class ByteTrackViewer : public rclcpp::Node
{
public:
    ByteTrackViewer(const std::string& node_name, const rclcpp::NodeOptions& options);
    ByteTrackViewer(const rclcpp::NodeOptions& options);
    ~ByteTrackViewer();

private:
    using Image = sensor_msgs::msg::Image;
    using BoundingBoxes = bboxes_ex_msgs::msg::BoundingBoxes;

    // Subscriptions
    image_transport::SubscriberFilter sub_image_;
    message_filters::Subscriber<BoundingBoxes> sub_bboxes_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, BoundingBoxes>;
    using ExactSyncPolicy = message_filters::sync_policies::ExactTime<Image, BoundingBoxes>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    using ExactSynchronizer = message_filters::Synchronizer<ExactSyncPolicy>;
    std::shared_ptr<Synchronizer> sync_;
    std::shared_ptr<ExactSynchronizer> exact_sync_;

    int queue_size_ = 5;
    bool use_exact_sync_ = false;
    std::string sub_image_topic_name_;
    std::string sub_bboxes_topic_name_;

    bool save_video_;
    int save_video_fps_;
    std::string save_video_name_;
    std::string save_video_codec_;
    cv::VideoWriter video_;

    void initializeParameter_();
    void connectCallback();

    void imageCallback(const Image::ConstSharedPtr& image_msg,
                       const BoundingBoxes::ConstSharedPtr& trackers_msg);
};
} // namespace bytetrack_viewer
