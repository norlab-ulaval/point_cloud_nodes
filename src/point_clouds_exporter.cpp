#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <sys/stat.h>

typedef PointMatcher<float> PM;

class PointCloudsExporter : public rclcpp::Node
{
public:
    PointCloudsExporter():
            Node("point_clouds_exporter")
    {
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<PointCloudsExporter>();

    node->declare_parameter<std::string>("bag_path", "");
    std::string bagPath = node->get_parameter("bag_path").as_string();
    if(bagPath.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "The bag_path parameter was not provided, exiting...");
        return 1;
    }

    node->declare_parameter<std::string>("topic_name", "");
    std::string topicName = node->get_parameter("topic_name").as_string();
    if(topicName.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "The topic_name parameter was not provided, exiting...");
        return 1;
    }

    node->declare_parameter<std::string>("output_folder", "");
    std::string outputFolder = node->get_parameter("output_folder").as_string();
    if(outputFolder.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "The output_folder parameter was not provided, exiting...");
        return 1;
    }

    mkdir(outputFolder.c_str(), 0775);

    std::shared_ptr<PM::DataPointsFilter> removeNanFilter = PM::get().DataPointsFilterRegistrar.create("RemoveNaNDataPointsFilter");

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
    sensor_msgs::msg::PointCloud2 msg;

    rosbag2_cpp::Reader reader;
    reader.open(bagPath);

    size_t topicMessageCount = 0;
    for(const rosbag2_storage::TopicInformation& topicInformation: reader.get_metadata().topics_with_message_count)
    {
        if(topicInformation.topic_metadata.name == topicName)
        {
            topicMessageCount = topicInformation.message_count;
            break;
        }
    }

    size_t messageCounter = 0;
    while(reader.has_next() && rclcpp::ok())
    {
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> serializedBagMessage = reader.read_next();
        if(serializedBagMessage->topic_name == topicName)
        {
            rclcpp::SerializedMessage serializedMessage(*serializedBagMessage->serialized_data);
            serialization.deserialize_message(&serializedMessage, &msg);
            std::string cloudFileName = "cloud_" + std::to_string(msg.header.stamp.sec) + "_" + std::to_string(msg.header.stamp.nanosec) + ".vtk";
            PM::DataPoints cloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<PM::ScalarType>(msg);
            removeNanFilter->inPlaceFilter(cloud);
            cloud.save(outputFolder + "/" + cloudFileName);
            RCLCPP_INFO_STREAM(node->get_logger(), "Progress: " << 100.0 * ++messageCounter / topicMessageCount << "%");
        }
    }

    rclcpp::shutdown();
    return 0;
}
