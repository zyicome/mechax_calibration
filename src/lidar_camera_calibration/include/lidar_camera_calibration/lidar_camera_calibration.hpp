#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>


#include <fstream>

class LidarCameraCalibration : public rclcpp::Node
{
public:
    LidarCameraCalibration();
};