#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pointmatcher_ros/PointMatcher_ROS.h>

typedef PointMatcher<float> PM;

class PointCloudExporter : public rclcpp::Node
{
public:
    PointCloudExporter():
            Node("point_cloud_exporter")
    {
        this->declare_parameter<std::string>("cloud_file_name", "cloud.vtk");
        this->get_parameter("cloud_file_name", cloudFileName);
        subscription =
                this->create_subscription<sensor_msgs::msg::PointCloud2>("points_in", 1, std::bind(&PointCloudExporter::callback, this, std::placeholders::_1));
    }

    void callback(const sensor_msgs::msg::PointCloud2& msg)
    {
        PM::DataPoints cloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<PM::ScalarType>(msg);
        RCLCPP_INFO_STREAM(this->get_logger(), "Saving cloud to " << cloudFileName);
        cloud.save(cloudFileName);
        rclcpp::shutdown();
    }

private:
    std::string cloudFileName;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
};

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudExporter>());
    return 0;
}

