#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pointmatcher_ros/PointMatcher_ROS.h>

typedef PointMatcher<float> PM;

class NanRemoverNode : public rclcpp::Node
{
public:
    NanRemoverNode():
            Node("NanRemoverNode")
    {
        nanFilter = PM::get().DataPointsFilterRegistrar.create("RemoveNaNDataPointsFilter");
        publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_out", 10);
        subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("points_in", 10, std::bind(&NanRemoverNode::callback, this, std::placeholders::_1));
    }

private:

    void callback(const sensor_msgs::msg::PointCloud2& msgIn)
    {
        PM::DataPoints pmCloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(msgIn);
        nanFilter->inPlaceFilter(pmCloud);
        sensor_msgs::msg::PointCloud2 msgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(pmCloud, msgIn.header.frame_id, msgIn.header.stamp);
        publisher->publish(msgOut);
    }

    std::shared_ptr<PM::DataPointsFilter> nanFilter;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NanRemoverNode>());
    rclcpp::shutdown();
    return 0;
}
