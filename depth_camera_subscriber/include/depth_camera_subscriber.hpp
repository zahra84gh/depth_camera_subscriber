#ifndef DEPTH_CAMERA_SUBSCRIBER_HPP_
#define DEPTH_CAMERA_SUBSCRIBER_HPP_

#include <iostream>
#include <vector>
#include <fstream>
#include <deque>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include "pcl/common/transforms.h"
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "sensor_msgs/msg/point_cloud.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

class DepthCameraSubscriber : public rclcpp::Node
{
public:
  DepthCameraSubscriber();
  void combined_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_in, const nav_msgs::msg::OccupancyGrid::SharedPtr map_in);

  void saveMapAsPGM(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, std::string file_path);
  std::array<float, 2> calculateMapOriginInPixel(const std::array<float, 2> &origin, float resolution, const std::array<int, 2> &image_size);
  bool findCameraToMapTransformation(std::string current_frame_id, std::string destination_frame_id, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform &transform);
  void getTransformedCloud(pcl::PointCloud<PointT>::Ptr cloud_output, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform &transform);
  std::array<float, 2> calculatePointPixelValue(const Eigen::Vector3f &position, const std::array<float, 2> &origin, float resolution);

  nav_msgs::msg::OccupancyGrid objectDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
  bool hasObject(const nav_msgs::msg::OccupancyGrid &map, int threshold);
  bool compareOccupancyGridsFromDifferentRobotPOV(const nav_msgs::msg::OccupancyGrid &grid1, const nav_msgs::msg::OccupancyGrid &grid2, double percentage_threshold);

  std::vector<float> findNearestOccupiedDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
  void writeDistancesToFile(const std::vector<float> &distances);

  geometry_msgs::msg::Point calculateCentroidInMapFrame(const nav_msgs::msg::OccupancyGrid &grid);
  geometry_msgs::msg::Point transformCentroidPointToRobotBase(const geometry_msgs::msg::Point &object_centroid_in_map_frame, geometry_msgs::msg::Transform map_to_robot_base_transform);
  double calculateDistance(const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2);
  double calculateAngle(const geometry_msgs::msg::Point &centroid_in_robot_base_frame);
  geometry_msgs::msg::Transform findMapToRobotBaseTransformation(geometry_msgs::msg::Transform &transform);
  void printMapToRobotBaseTransform(geometry_msgs::msg::Transform &transform);
  geometry_msgs::msg::Point transformImgCoordinateToRos(double o_x, double o_y, double resolution, double height, double x, double y);
  void calculateObjectTypeCount();
  void determineObjectType(const std::vector<int> &counts);
  
  nav_msgs::msg::OccupancyGrid::SharedPtr updateLocalMap(const nav_msgs::msg::OccupancyGrid &dynamic_object_map, const nav_msgs::msg::OccupancyGrid::SharedPtr &static_map);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_projected_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_filtered_distance_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_transformed_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_in_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_transformed_member_publisher_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr detected_object_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr buffer_modified_map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr updated_map_publisher_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_in_;
  bool map_data_set = false;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_member_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_member_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::Transform transform_1; // camera to map transformation

  rclcpp::Rate rate{0.1};
  std::mutex lock_;
  bool is_transformed_ = false;
  bool continue_callback_ = true;
  std::array<float, 2> origin_pixel; // map origin in pixel

  std::deque<nav_msgs::msg::OccupancyGrid> modified_map_buffer_;
  int buffer_size = 2;
  rclcpp::Time timer_stamp_;
  geometry_msgs::msg::Point first_time_object_seen_centroid_ros_;
  geometry_msgs::msg::Transform transform_2; // map to robot base transformation
  nav_msgs::msg::OccupancyGrid main_object_modified_map_;
  std::vector<int> count_;
};

#endif
