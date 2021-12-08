#include "bytetrack_cpp_node/bytetrack_cpp_node.hpp"
#include "bytetrack_cpp_node/coco_names.hpp"

namespace bytetrack_cpp_node{
    using namespace bytetrack_cpp;

    std::vector<Object> BoundingBoxes2Objects(const std::vector<bboxes_ex_msgs::msg::BoundingBox> bboxes)
    {
        std::vector<Object> objects;
        float scale = 1.0;
        for(auto bbox: bboxes){
            Object obj;
            obj.rect.x = (bbox.xmin) / scale;
            obj.rect.y = (bbox.ymin) / scale;
            obj.rect.width = (bbox.xmax - bbox.xmin) / scale;
            obj.rect.height = (bbox.ymax - bbox.ymin) / scale;

            auto it = std::find(COCO_CLASSES, COCO_CLASSES + 80, bbox.class_id);
            if (it != COCO_CLASSES + 80){
                int idx = std::distance(COCO_CLASSES, it);
                std::cout << idx << std::endl;
                obj.label = idx;
            }
            
            obj.prob = bbox.probability;
            objects.push_back(obj);
        }
        return objects;
    }
    std::vector<bboxes_ex_msgs::msg::BoundingBox> STrack2BoundingBoxes(const std::vector<STrack> trackers)
    {
        std::vector<bboxes_ex_msgs::msg::BoundingBox> bboxes;
        for(int i=0; i<trackers.size(); i++){
            bboxes_ex_msgs::msg::BoundingBox bbox;
            // bbox.class_id = trackers[i].
            bbox.ymin = trackers[i].tlbr[0];
            bbox.xmin = trackers[i].tlbr[1];
            bbox.ymax = trackers[i].tlbr[2];
            bbox.xmax = trackers[i].tlbr[3];
            bbox.id = trackers[i].track_id;
            // bbox.class_id = "";
            // bbox.center_dist = 0.0;
            // bbox.img_height = 0;
            // bbox.img_width = 0;
            bboxes.push_back(bbox);
        }
        return bboxes;
    }
    
    ByteTrackNode::ByteTrackNode(const std::string &node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node("bytetrack_cpp_node", node_name, options)
    {
    }
    ByteTrackNode::ByteTrackNode(const rclcpp::NodeOptions& options)
    : ByteTrackNode("", options)
    {
        this->initializeParameter_();
        this->tracker_ = std::make_unique<BYTETracker>(this->video_fps_, this->track_buffer_);

        this->sub_bboxes_ = this->create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>(
                            this->sub_bboxes_topic_name_, 10, 
                            std::bind(&ByteTrackNode::topic_callback_, 
                                      this,
                                      std::placeholders::_1));
        this->pub_bboxes_ = this->create_publisher<bboxes_ex_msgs::msg::BoundingBoxes>(
            this->pub_bboxes_topic_name_,
            10
        );
    }
    void ByteTrackNode::initializeParameter_()
    {
        this->video_fps_ = this->declare_parameter<int>("video_fps", 30);
        this->track_buffer_ = this->declare_parameter<int>("track_buffer", 30);
        this->sub_bboxes_topic_name_ = this->declare_parameter<std::string>("sub_bboxes_topic_name", "yolox/bounding_boxes");
        this->pub_bboxes_topic_name_ = this->declare_parameter<std::string>("pub_bboxes_topic_name", "bytetrack/bounding_boxes");
    }
    void ByteTrackNode::topic_callback_(const bboxes_ex_msgs::msg::BoundingBoxes::ConstSharedPtr msg)
    {
        bboxes_ex_msgs::msg::BoundingBoxes bboxes;
        bboxes.header = msg->header;
        bboxes.image_header = msg->image_header;

        vector<Object> objects = BoundingBoxes2Objects(msg->bounding_boxes);
        vector<STrack> output_stracks = this->tracker_->update(objects);
        std::cout << "Object count = " << objects.size() << ", STrack count = " << output_stracks.size() << std::endl;
        bboxes.bounding_boxes = STrack2BoundingBoxes(output_stracks);
        this->pub_bboxes_->publish(bboxes);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<bytetrack_cpp_node::ByteTrackNode>(node_options));
  rclcpp::shutdown();
  return 0;
}
RCLCPP_COMPONENTS_REGISTER_NODE(bytetrack_cpp_node::ByteTrackNode)