#include "utils.h"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>

class LivoxPointCloudToPointCloud2Node : public rclcpp::Node
{
public:
  LivoxPointCloudToPointCloud2Node()
    : Node("livox_pointcloud_to_pointcloud2_node")
  {
    _sub_livox_pointcloud = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar",
      rclcpp::QoS(2).best_effort(),
      std::bind(&LivoxPointCloudToPointCloud2Node::callbackLivoxPointCloud, this, std::placeholders::_1)
    );

    _pub_pointcloud2 = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/livox/pointcloud2",
      rclcpp::QoS(2).reliable()
    );
  }

private:
  void callbackLivoxPointCloud(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    auto pointcloud2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    const auto cloud = Utils::livox2PCL(msg, 1, 0.0, 100.0);
    
    pcl::toROSMsg(*cloud, *pointcloud2_msg);
    pointcloud2_msg->header = msg->header;

    _pub_pointcloud2->publish(*pointcloud2_msg);
  }

  std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> _sub_livox_pointcloud;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> _pub_pointcloud2;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoxPointCloudToPointCloud2Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
