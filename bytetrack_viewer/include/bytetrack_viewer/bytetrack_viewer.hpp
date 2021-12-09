#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "bboxes_ex_msgs/msg/bounding_box.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"

namespace bytetrack_viewer{

class ByteTrackViewer : public rclcpp::Node
{
    public:
        ByteTrackViewer(const std::string &node_name, const rclcpp::NodeOptions& options);
        ByteTrackViewer(const rclcpp::NodeOptions& options);

    private:
        using Image = sensor_msgs::msg::Image;
        using BoundingBoxes = bboxes_ex_msgs::msg::BoundingBoxes;

        // Subscriptions
        image_transport::SubscriberFilter sub_image_;
        message_filters::Subscriber<BoundingBoxes> sub_bboxes_;
        using SyncPolicy =
            message_filters::sync_policies::ApproximateTime<Image, BoundingBoxes>;
        using ExactSyncPolicy =
            message_filters::sync_policies::ExactTime<Image, BoundingBoxes>;
        using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
        using ExactSynchronizer = message_filters::Synchronizer<ExactSyncPolicy>;
        std::shared_ptr<Synchronizer> sync_;
        std::shared_ptr<ExactSynchronizer> exact_sync_;

        std::mutex connect_mutex_;
        int queue_size_ = 5;
        bool use_exact_sync_ = false;
        std::string sub_image_topic_name_;
        std::string sub_bboxes_topic_name_;

        void initializeParameter_();
        void connectCallback();

        void imageCallback(
            const Image::ConstSharedPtr & image_msg,
            const BoundingBoxes::ConstSharedPtr & trackers_msg);
};
}