#include "bytetrack_viewer/bytetrack_viewer.hpp"

namespace bytetrack_viewer
{
cv::Scalar getColor(int id)
{
    int idx = id + 3;
    return cv::Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}

void drawObject(cv::Mat &frame, const bboxes_ex_msgs::msg::BoundingBox &bbox)
{
    // draw bbox
    const auto color = getColor(bbox.id);
    cv::rectangle(frame,
                  //   cv::Rect(bbox.ymin, bbox.xmin, bbox.ymax - bbox.ymin, bbox.xmax - bbox.xmin),
                  cv::Rect(bbox.xmin, bbox.ymin, bbox.xmax - bbox.xmin, bbox.ymax - bbox.ymin),
                  color, 2);

    // draw ID
    const double brightness = color[2] * 0.3 + color[1] * 0.59 + color[0] * 0.11;
    const cv::Scalar txt_color =
        (brightness > 127) ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255);

    const std::string txt = cv::format("ID:%d %s", bbox.id, bbox.class_id.c_str());
    int baseLine = 0;
    const cv::Size label_size = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseLine);
    cv::rectangle(frame,
                  cv::Rect(cv::Point(bbox.ymin, bbox.xmin - label_size.height),
                           cv::Size(label_size.width, label_size.height + baseLine)),
                  color, -1);
    cv::putText(frame, txt, cv::Point(bbox.ymin, bbox.xmin), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                txt_color, 1, cv::LINE_AA);
}

ByteTrackViewer::ByteTrackViewer(const std::string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp::Node("bytetrack_viewer", node_name, options)
{
}
ByteTrackViewer::ByteTrackViewer(const rclcpp::NodeOptions &options) : ByteTrackViewer("", options)
{
    this->initializeParameter_();

    if (this->use_exact_sync_)
    {
        this->exact_sync_ = std::make_shared<ExactSynchronizer>(ExactSyncPolicy(this->queue_size_),
                                                                sub_image_, sub_bboxes_);
        this->exact_sync_->registerCallback(std::bind(
            &ByteTrackViewer::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
        this->sync_ =
            std::make_shared<Synchronizer>(SyncPolicy(this->queue_size_), sub_image_, sub_bboxes_);
        this->sync_->registerCallback(std::bind(&ByteTrackViewer::imageCallback, this,
                                                std::placeholders::_1, std::placeholders::_2));
    }

    connectCallback();
    cv::namedWindow("ByteTrackViewer", cv::WINDOW_AUTOSIZE);
}

ByteTrackViewer::~ByteTrackViewer()
{
    if (this->video_.isOpened())
    {
        this->video_.release();
        RCLCPP_INFO_STREAM(this->get_logger(), "save as " << this->save_video_name_ << ".");
    }
}

void ByteTrackViewer::initializeParameter_()
{
    this->queue_size_ = this->declare_parameter<int>("queue_size", 5);
    this->use_exact_sync_ = this->declare_parameter<bool>("exact_sync", false);
    this->sub_image_topic_name_ =
        this->declare_parameter<std::string>("sub_image_topic_name", "/image_raw");
    this->sub_bboxes_topic_name_ =
        this->declare_parameter<std::string>("sub_bboxes_topic_name", "/bytetrack/bounding_boxes");
    this->save_video_ = this->declare_parameter<bool>("save_video", false);
    this->save_video_fps_ = this->declare_parameter<int>("save_video_fps", 30);
    this->save_video_name_ = this->declare_parameter<std::string>("save_video_name", "output.avi");
    this->save_video_codec_ = this->declare_parameter<std::string>("save_video_codec", "MJPG");
}

void ByteTrackViewer::connectCallback()
{
    image_transport::TransportHints hints(this, "raw");
    this->sub_image_.subscribe(this, this->sub_image_topic_name_, hints.getTransport());
    this->sub_bboxes_.subscribe(this, this->sub_bboxes_topic_name_);
}

void ByteTrackViewer::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
    const bboxes_ex_msgs::msg::BoundingBoxes::ConstSharedPtr &trackers_msg)
{
    auto img = cv_bridge::toCvCopy(image_msg, "bgr8");
    cv::Mat frame = img->image;

    if (this->save_video_ && !this->video_.isOpened())
    {
        this->video_ = cv::VideoWriter(this->save_video_name_,
                                       cv::VideoWriter::fourcc(this->save_video_codec_.c_str()[0],
                                                               this->save_video_codec_.c_str()[1],
                                                               this->save_video_codec_.c_str()[2],
                                                               this->save_video_codec_.c_str()[3]),
                                       this->save_video_fps_, frame.size(), true);
    }

    for (const auto &bbox : trackers_msg->bounding_boxes)
    {
        drawObject(frame, bbox);
    }
    if (this->save_video_)
    {
        this->video_ << frame;
    }
    cv::imshow("ByteTrackViewer", frame);
    auto key = cv::waitKey(1);
    if (key == 27)
    {
        rclcpp::shutdown();
    }
}
} // namespace bytetrack_viewer

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    rclcpp::spin(std::make_shared<bytetrack_viewer::ByteTrackViewer>(node_options));
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bytetrack_viewer::ByteTrackViewer)
