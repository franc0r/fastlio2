#include "utils.h"

#include <optional>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#ifdef LIVOX_ROS_DRIVER2
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = msg->point_num;
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num)
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {

            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / 1000000.0f;
            cloud->push_back(p);
        }
    }
    return cloud;
}
#endif

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::pointCloud2ToPCL(
    const sensor_msgs::msg::PointCloud2 &msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    const size_t point_num = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    if (filter_num < 1)
        filter_num = 1;
    cloud->reserve(point_num / static_cast<size_t>(filter_num) + 1);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(msg, "intensity");

    const float min_range_sq = static_cast<float>(min_range * min_range);
    const float max_range_sq = static_cast<float>(max_range * max_range);

    for (size_t i = 0; iter_x != iter_x.end(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
        // filter out points based on filter_num
        if ((i % static_cast<size_t>(filter_num)) != 0) {
            continue;
        }
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        const float intensity = *iter_intensity;
        const float range_sq = x * x + y * y + z * z;

        // filter out points based on range
        if (range_sq < min_range_sq || range_sq > max_range_sq) {
            continue;
        }

        // valid point, add to cloud
        pcl::PointXYZINormal p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = intensity;
        p.curvature = 0.0f;
        cloud->push_back(p);
    }

    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
