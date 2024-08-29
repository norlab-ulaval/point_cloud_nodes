#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <rclcpp_components/register_node_macro.hpp>

typedef PointMatcher<float> PM;

class PointCloudDownsampler : public rclcpp::Node
{
public:
    PointCloudDownsampler(const rclcpp::NodeOptions & options):
            Node("point_cloud_downsampler", options)
    {
        cloudCounter = 0;

        this->declare_parameter<double>("rate_downsampling", 1.0);
        double rateDownsampling = this->get_parameter("rate_downsampling").as_double();
        cloudCounterPeriod = int(1.0 / rateDownsampling);

        this->declare_parameter<double>("point_downsampling", 1.0);
        double pointDownsampling = this->get_parameter("point_downsampling").as_double();
        PM::Parameters randomSamplingFilterParams;
        randomSamplingFilterParams["prob"] = std::to_string(pointDownsampling);
        randomSamplingFilter = PM::get().DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", randomSamplingFilterParams);

        publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_out", 10);

        subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("points_in", 10, std::bind(&PointCloudDownsampler::callback, this, std::placeholders::_1));
    }

    void callback(const sensor_msgs::msg::PointCloud2& msgIn)
    {
        if(cloudCounter == 0)
        {
            PM::DataPoints cloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<PM::ScalarType>(msgIn);
            randomSamplingFilter->inPlaceFilter(cloud);
            sensor_msgs::msg::PointCloud2 msgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<PM::ScalarType>(cloud, msgIn.header.frame_id, msgIn.header.stamp);
            publisher->publish(msgOut);
        }
        cloudCounter = (cloudCounter + 1) % cloudCounterPeriod;
    }

private:
    int cloudCounter;
    int cloudCounterPeriod;
    std::shared_ptr<PM::DataPointsFilter> randomSamplingFilter;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
};

RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudDownsampler)

