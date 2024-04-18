#include "depth_camera_subscriber.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

DepthCameraSubscriber::DepthCameraSubscriber()
    : Node("depth_camera_subscriber"), tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/right_camera/depth/color/points", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr pc_in)
      {
        this->combined_callback(pc_in, nullptr);
      });

  map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr map_in)
      {
        this->combined_callback(nullptr, map_in);
      });

  cloud_in_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_in", rclcpp::SensorDataQoS());
  cloud_projected_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_projected", rclcpp::SensorDataQoS());
  cloud_filtered_distance_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered", rclcpp::SensorDataQoS());
  cloud_transformed_member_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_transformed", rclcpp::SensorDataQoS());

  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("cloud_map_in", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  detected_object_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("detected_object_grid", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  buffer_modified_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("buffer_modified_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  updated_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("updated_map_grid", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void DepthCameraSubscriber::combined_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_in, const nav_msgs::msg::OccupancyGrid::SharedPtr map_in)
{
  if (pc_in != nullptr)
  {
    sensor_msgs::msg::PointCloud2 output;
    output.header = pc_in->header;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_in, *cam_cloud_in); // Convert ROS message to PC

    pcl::toROSMsg(*cam_cloud_in, output); // Convert PC to ROS message
    cloud_in_publisher_->publish(output);

    pcl::io::savePCDFileASCII("src/depth_camera_subscriber/param/pc_save_XYZ.pcd", *cam_cloud_in);

    // Convert 3D point Clouds to 2D projected plane
    PointCloudT::Ptr cloud_projected(new PointCloudT);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); // Create a set of planar coefficients with X=Z=0,Y=1 (Ax + By + Cz + D = 0)
    coefficients->values.resize(4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 1.0;
    coefficients->values[2] = 0;
    coefficients->values[3] = 0;
    pcl::ProjectInliers<pcl::PointXYZ> projection;
    projection.setModelType(pcl::SACMODEL_PLANE);
    projection.setInputCloud(cam_cloud_in);
    projection.setModelCoefficients(coefficients);
    projection.filter(*cloud_projected);
    pcl::io::savePCDFileASCII("src/depth_camera_subscriber/param/cloud_projected.pcd", *cloud_projected);

    pcl::toROSMsg(*cloud_projected, output);
    cloud_projected_publisher_->publish(output);

    // Filter points which are located more than 2 meters far away
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    float max_distance_threshold = 2.0;
    for (const pcl::PointXYZ &point : cloud_projected->points)
    {
      float distance_2d = std::sqrt(point.x * point.x + point.z * point.z);
      if (distance_2d <= max_distance_threshold)
      {
        cloud_filtered->push_back(point);
      }
    }

    // Publish filtered point cloud
    sensor_msgs::msg::PointCloud2 output_1;
    pcl::toROSMsg(*cloud_filtered, output_1);
    output_1.header = pc_in->header;
    cloud_filtered_distance_publisher_->publish(output_1);

    cloud_filtered_member_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    *cloud_filtered_member_ = *cloud_filtered;
  }

  if (map_in != nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "Map is loaded");
    std::string map_path = "src/depth_camera_subscriber/param/map.pgm";
    //saveMapAsPGM(map_in, map_path);

    // Publish static map
    nav_msgs::msg::OccupancyGrid map_output;
    map_output.header = map_in->header;
    map_output.info = map_in->info;
    map_output.data = map_in->data;
    map_publisher_->publish(map_output);
    map_data_set = true;

    std::array<float, 2> origin = {map_in->info.origin.position.x, map_in->info.origin.position.y};
    float resolution = map_in->info.resolution;

    std::array<int, 2> map_size = {map_in->info.width, map_in->info.height};
    int map_width = map_in->info.width;
    int map_height = map_in->info.height;

    std::array<float, 2> map_origin_pixel = calculateMapOriginInPixel(origin, resolution, map_size);
    origin_pixel = {map_origin_pixel[0], map_origin_pixel[1]};
    RCLCPP_INFO(
        this->get_logger(),
        "map-origin_pixel_x: %.f, map-origin_pixel_y: %.f",
        map_origin_pixel[0], map_origin_pixel[1]);
    map_in_ = map_in;
  }

  if (pc_in != nullptr && map_data_set)
  {
    PointCloudT::Ptr cloud_transformed(new PointCloudT);
    const std::string &destination_frame_id = "map";
    const std::string &current_frame_id = "right_camera_depth_optical_frame";
    is_transformed_ = findCameraToMapTransformation(current_frame_id, destination_frame_id, cloud_transformed, transform_1);
    if (continue_callback_)
    {
      // Publish transformed point cloud
      *cloud_transformed_member_ = *cloud_transformed;
      sensor_msgs::msg::PointCloud2 output_2;
      output_2.header = pc_in->header;
      pcl::toROSMsg(*cloud_transformed_member_, output_2);
      cloud_transformed_member_publisher_->publish(output_2);

      std::vector<float> distances = findNearestOccupiedDistance(cloud_transformed_member_);
      writeDistancesToFile(distances);

      // Publish detected object grid
      nav_msgs::msg::OccupancyGrid modified_map;
      modified_map = objectDetection(cloud_transformed_member_);
      detected_object_publisher_->publish(modified_map);

      calculateObjectTypeCount();
    }
  }
  rate.sleep();
} // End of callback

void DepthCameraSubscriber::saveMapAsPGM(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, std::string file_path)
{
  std::ofstream pgm_file(file_path, std::ios::out | std::ios::binary);

  if (pgm_file.is_open())
  {
    pgm_file << "P5\n";
    pgm_file << map_msg->info.width << " " << map_msg->info.height << "\n";
    pgm_file << "255\n";

    for (int i = 0; i < map_msg->info.width * map_msg->info.height; ++i)
    {
      uint8_t pixel_value = 255 - map_msg->data[i];
      pgm_file.write(reinterpret_cast<const char *>(&pixel_value), sizeof(pixel_value));
    }

    pgm_file.close();
    RCLCPP_INFO(this->get_logger(), "Map saved as map.pgm");
  }
  else
  {
    std::cerr << "Error: Unable to open file for writing." << std::endl;
  }
}

// Calculate the static map origin in pixel value
std::array<float, 2> DepthCameraSubscriber::calculateMapOriginInPixel(const std::array<float, 2> &origin, float resolution, const std::array<int, 2> &map_size)
{
  float x = -1 * origin[0] / resolution;
  float y = map_size[1] + (origin[1] / resolution); // y axis is flipped

  return {x, y};
}

bool DepthCameraSubscriber::findCameraToMapTransformation(std::string current_frame_id, std::string destination_frame_id, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform &transform)
{
  if (cloud_filtered_member_->points.size() == 0)
  {
    rclcpp::get_logger("pointcloud empty / Pointcloud not recieved.");
    return false;
  }
  try
  {
    transform = tf_buffer_->lookupTransform(destination_frame_id, current_frame_id, tf2::TimePointZero).transform;
    std::scoped_lock lock(lock_);
    getTransformedCloud(cloud_filtered_member_, cloud_transformed, transform);
    cloud_transformed->header.frame_id = destination_frame_id;
    continue_callback_ = true;
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Exception in lookupTransform: %s", ex.what());
    continue_callback_ = false;
  }
  if (!continue_callback_)
    return false;
}

// Transform PC from camera to map frame
void DepthCameraSubscriber::getTransformedCloud(pcl::PointCloud<PointT>::Ptr cloud_filtered_member_, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform &transform)
{
  Eigen::Quaternion<float> rotation(transform.rotation.w,
                                    transform.rotation.x,
                                    transform.rotation.y,
                                    transform.rotation.z);

  Eigen::Vector3f translation(transform.translation.x,
                              transform.translation.y,
                              transform.translation.z);

  if (cloud_filtered_member_->size() < 1)
  {
    cloud_transformed = cloud_filtered_member_;
    return;
  }
  pcl::transformPointCloud(*cloud_filtered_member_, *cloud_transformed, translation, rotation);

  cloud_transformed_member_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  *cloud_transformed_member_ = *cloud_transformed;
}

// Calculate the corresponding pixel cell for each point of object
std::array<float, 2> DepthCameraSubscriber::calculatePointPixelValue(const Eigen::Vector3f &position, const std::array<float, 2> &origin, float resolution)
{
  float delta_x = position[0] / resolution;
  float delta_y = position[1] / resolution;

  float x = origin[0] + delta_x;
  float y = origin[1] + (-1 * delta_y);

  return {x, y};
}

nav_msgs::msg::OccupancyGrid DepthCameraSubscriber::objectDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  nav_msgs::msg::OccupancyGrid modified_map;
  modified_map.info = map_in_->info;
  modified_map.header = map_in_->header;
  modified_map.data.resize(map_in_->info.width * map_in_->info.height, 0);

  for (const pcl::PointXYZ &point : cloud->points)
  {
    Eigen::Vector3f position(point.x, point.y, point.z);
    float resolution = map_in_->info.resolution;
    std::array<float, 2> point_pixel = calculatePointPixelValue(position, origin_pixel, resolution);

    int x = static_cast<int>(point_pixel[0]);
    int y = static_cast<int>(point_pixel[1]);

    if (x >= 1 && x < map_in_->info.width - 1 && y >= 1 && y < map_in_->info.height - 1)
    {
      bool isOccupied = false;

      // Check 2 neighboring points
      for (int dx = -2; dx <= 2; ++dx)
      {
        for (int dy = -2; dy <= 2; ++dy)
        {
          int nx = x + dx;
          int ny = y + dy;

          int index = map_in_->info.width * (map_in_->info.height - ny - 1) + nx;
          if (map_in_->data[index] == 100 | map_in_->data[index] == -1)
          {
            isOccupied = true;
            break;
          }
        }
      }

      if (isOccupied)
      {
        int index = map_in_->info.width * (map_in_->info.height - y - 1) + x;
        modified_map.data[index] = 0;
      }

      else
      {
        int index = map_in_->info.width * (map_in_->info.height - y - 1) + x;
        modified_map.data[index] = 100;
      }
    }
  }
  return modified_map;
}

std::vector<float> DepthCameraSubscriber::findNearestOccupiedDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  std::vector<float> distances;
  for (const pcl::PointXYZ &point : cloud->points)
  {
    Eigen::Vector3f position(point.x, point.y, point.z);
    float resolution = map_in_->info.resolution;
    std::array<float, 2> point_pixel = calculatePointPixelValue(position, origin_pixel, resolution);

    int x = static_cast<int>(point_pixel[0]);
    int y = static_cast<int>(point_pixel[1]);

    float min_distance = std::numeric_limits<float>::max();

    for (int dx = -20; dx <= 20; ++dx)
    {
      for (int dy = -20; dy <= 20; ++dy)
      {
        int nx = x + dx;
        int ny = y + dy;

        if (nx >= 0 && nx < map_in_->info.width && ny >= 0 && ny < map_in_->info.height)
        {
          int index = map_in_->info.width * (map_in_->info.height - ny - 1) + nx;

          if (map_in_->data[index] == 100)
          {
            float distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
            if (distance < min_distance)
            {
              min_distance = distance;
            }
          }
        }
      }
    }

    if (min_distance != std::numeric_limits<float>::max())
    {
      distances.push_back(min_distance);
    }
  }
  return distances;
}

void DepthCameraSubscriber::writeDistancesToFile(const std::vector<float> &distances)
{
  const std::string &file_path = "src/depth_camera_subscriber/param/distances.txt";
  std::ofstream outputFile(file_path);

  if (outputFile.is_open())
  {
    for (const float &distance : distances)
    {
      outputFile << distance << "\n";
    }
    outputFile.close();
    // RCLCPP_INFO(this->get_logger(), "Distances are written to distances.txt");
  }
  else
  {
    std::cerr << "Unable to open file for writing" << std::endl;
  }
}

bool DepthCameraSubscriber::hasObject(const nav_msgs::msg::OccupancyGrid &occupancy_grid, int threshold)
{
  int occupied_count = 0;
  for (int i = 0; i < occupancy_grid.data.size(); ++i)
  {
    if (occupancy_grid.data[i] == 100)
    {
      occupied_count++;
    }
  }
  if (occupied_count >= threshold)
  {
    RCLCPP_INFO(this->get_logger(), "Map has an object with %d pixels.", occupied_count);
    return true;
  }
  RCLCPP_INFO(this->get_logger(), "Map with %d pixels is considered empty.", occupied_count);
  return false;
}

// Compares two occupancy grids, typically representing the same object observed from different robot points of view.
bool DepthCameraSubscriber::compareOccupancyGridsFromDifferentRobotPOV(const nav_msgs::msg::OccupancyGrid &grid1, const nav_msgs::msg::OccupancyGrid &grid2, double percentage_threshold)
{
  if (grid1.info.width != grid2.info.width || grid1.info.height != grid2.info.height)
  {
    return false;
  }

  int counter = 0;

  int occupied_count_grid1 = 0;
  int occupied_count_grid2 = 0;

  for (int i = 0; i < grid1.data.size(); ++i)
  {
    if (grid1.data[i] == 100)
    {
      occupied_count_grid1++;
    }
  }

  for (int i = 0; i < grid2.data.size(); ++i)
  {
    if (grid2.data[i] == 100)
    {
      occupied_count_grid2++;
    }
  }

  int common_threshold = std::round(std::min(occupied_count_grid1, occupied_count_grid2) * (percentage_threshold / 100.0));
  RCLCPP_INFO(this->get_logger(), "common_threshold is %d pixels.", common_threshold);

  if (occupied_count_grid1 > occupied_count_grid2)
  {
    for (int i = 1; i < grid2.info.width - 1; ++i)
    {
      for (int j = 1; j < grid2.info.height - 1; ++j)
      {
        int index_2 = grid2.info.width * j + i;

        if (grid2.data[index_2] == 100)
        {
          int x = i;
          int y = grid2.info.height - j - 1;

          bool foundOccupied = false;

          for (int dx = -occupied_count_grid1; dx <= occupied_count_grid1; ++dx)
          {
            for (int dy = -occupied_count_grid1; dy <= occupied_count_grid1; ++dy)
            {
              int nx = x + dx;
              int ny = y + dy;

              if (nx >= 0 && nx < grid1.info.width && ny >= 0 && ny < grid1.info.height)
              {
                int index_1 = grid1.info.width * (grid1.info.height - ny - 1) + nx;

                if (grid1.data[index_1] == 100)
                {
                  foundOccupied = true;
                  break;
                }
              }
            }
            if (foundOccupied)
            {
              break;
            }
          }

          if (foundOccupied)
          {
            counter++;
          }
        }
      }
    }
  }
  else
  {
    for (int i = 1; i < grid1.info.width - 1; ++i)
    {
      for (int j = 1; j < grid1.info.height - 1; ++j)
      {
        int index_1 = grid1.info.width * j + i;

        if (grid1.data[index_1] == 100)
        {
          int x = i;
          int y = grid1.info.height - j - 1;

          bool foundOccupied = false;

          for (int dx = -occupied_count_grid2; dx <= occupied_count_grid2; ++dx)
          {
            for (int dy = -occupied_count_grid2; dy <= occupied_count_grid2; ++dy)
            {
              int nx = x + dx;
              int ny = y + dy;

              if (nx >= 0 && nx < grid2.info.width && ny >= 0 && ny < grid2.info.height)
              {
                int index_2 = grid2.info.width * (grid2.info.height - ny - 1) + nx;

                if (grid2.data[index_2] == 100)
                {
                  foundOccupied = true;
                  break;
                }
              }
            }
            if (foundOccupied)
            {
              break;
            }
          }

          if (foundOccupied)
          {
            counter++;
          }
        }
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Occupied pixels in grid1: %d pixels.", occupied_count_grid1);
  RCLCPP_INFO(this->get_logger(), "Occupied pixels in grid2: %d pixels.", occupied_count_grid2);
  RCLCPP_INFO(this->get_logger(), "Number of common pixels is: %d (%.f%%)", counter, (static_cast<double>(std::min(counter, common_threshold)) / common_threshold) * 100);
  return counter >= common_threshold;
}

// Calculate the object centroid in map frame
geometry_msgs::msg::Point DepthCameraSubscriber::calculateCentroidInMapFrame(const nav_msgs::msg::OccupancyGrid &grid)
{
  geometry_msgs::msg::Point centroid;
  int total_occupied_cell = 0;
  float sum_x = 0.0;
  float sum_y = 0.0;

  for (int y = 0; y < grid.info.height; ++y)
  {
    for (int x = 0; x < grid.info.width; ++x)
    {
      int index = grid.info.width * (grid.info.height - y - 1) + x;
      if (grid.data[index] == 100)
      {
        sum_x += x;
        sum_y += y;
        ++total_occupied_cell;
      }
    }
  }
  if (total_occupied_cell > 0)
  {
    centroid.x = sum_x / total_occupied_cell;
    centroid.y = sum_y / total_occupied_cell;
    centroid.z = 0.0;
  }
  return centroid;
}

// Convert image coordinate (static map) to ROS coordinate
geometry_msgs::msg::Point DepthCameraSubscriber::transformImgCoordinateToRos(double map_origin_x, double map_origin_y, double map_resolution, double map_height, double pixel_x, double pixel_y)
{
  double x = static_cast<double>(pixel_x);
  double y = static_cast<double>(pixel_y);
  double o_x = static_cast<double>(map_origin_x);
  double o_y = static_cast<double>(map_origin_y);
  double resolution = static_cast<double>(map_resolution);
  double height = static_cast<double>(map_height);

  geometry_msgs::msg::Point object_centroid_in_ros_map_frame;
  object_centroid_in_ros_map_frame.x = round(x * resolution + o_x);
  object_centroid_in_ros_map_frame.y = round(height * resolution - y * resolution + o_y);

  return object_centroid_in_ros_map_frame;
}

geometry_msgs::msg::Transform DepthCameraSubscriber::findMapToRobotBaseTransformation(geometry_msgs::msg::Transform &transform)
{
  try
  {
    std::string current_frame_id = "map";
    std::string destination_frame_id = "base_link";
    transform = tf_buffer_->lookupTransform(destination_frame_id, current_frame_id, tf2::TimePointZero).transform;
    return transform;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Exception in lookupTransform: %s", ex.what());
  }
}

void DepthCameraSubscriber::printMapToRobotBaseTransform(geometry_msgs::msg::Transform &transform)
{
  Eigen::Quaternion<float> rotation(transform.rotation.w,
                                    transform.rotation.x,
                                    transform.rotation.y,
                                    transform.rotation.z);

  Eigen::Vector3f translation(transform.translation.x,
                              transform.translation.y,
                              transform.translation.z);

  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  transform_matrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  transform_matrix.block<3, 1>(0, 3) = translation;

  std::cout << "Map To Robot Base Transformation Matrix" << std::endl;
  RCLCPP_INFO(this->get_logger(), "Map To Robot Base Transformation Matrix", transform_matrix);
}

geometry_msgs::msg::Point DepthCameraSubscriber::transformCentroidPointToRobotBase(const geometry_msgs::msg::Point &object_centroid_in_map_frame, geometry_msgs::msg::Transform transform)
{
  Eigen::Vector4d object_centroid_vector;
  object_centroid_vector << object_centroid_in_map_frame.x, object_centroid_in_map_frame.y, 0.0, 1.0;

  Eigen::Quaterniond rotation(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  Eigen::Vector3d translation(transform.translation.x, transform.translation.y, transform.translation.z);

  Eigen::Affine3d transform_affine = Eigen::Translation3d(translation) * rotation;

  // Apply transformation to the object centroid
  Eigen::Vector4d point_in_robot_link_frame = transform_affine * object_centroid_vector;

  geometry_msgs::msg::Point point_in_robot_link_frame_msg;
  point_in_robot_link_frame_msg.x = point_in_robot_link_frame(0);
  point_in_robot_link_frame_msg.y = point_in_robot_link_frame(1);
  point_in_robot_link_frame_msg.z = point_in_robot_link_frame(2);

  return point_in_robot_link_frame_msg;
}

// Calculate distance of object centroid to robot base
double DepthCameraSubscriber::calculateDistance(const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2)
{
  double dx = point1.x - point2.x;
  double dy = point1.y - point2.y;
  double dz = point1.z - point2.z;

  return sqrt(dx * dx + dy * dy + dz * dz);
}

// Calculate the angle of object centroid to robot base
double DepthCameraSubscriber::calculateAngle(const geometry_msgs::msg::Point &centroid_in_robot_base_frame)
{
  // Calculate the vector components
  double vector_x = centroid_in_robot_base_frame.x;
  double vector_y = centroid_in_robot_base_frame.y;

  double angle = std::atan2(vector_y, vector_x);

  // Convert rad to degrees
  angle = angle * (180.0 / M_PI);

  return angle;
}

//Determine object type depends on the counter
void DepthCameraSubscriber::determineObjectType(const std::vector<int> &counts)
{

  for (size_t i = 0; i < counts.size(); ++i)
  {
    RCLCPP_INFO(this->get_logger(), "Counter[%zu]: %d", i, counts[i]);
  }
  int total_count = 0;
  for (const auto &count : counts)
  {
    total_count += count;
  }
  if (total_count == 1)
  {
    RCLCPP_INFO(this->get_logger(), "The object belongs to Group 1, including people or other robots.");
  }
  else if (total_count == 2)
  {
    RCLCPP_INFO(this->get_logger(), "The object belongs to Group 2, including trolley, forklift or shopping cart.");
  }
  else if (total_count >= 8)
  {
    RCLCPP_INFO(this->get_logger(), "The object is identified as a static entity, and the map should be updated by adding this object into it.");
    map_in_ = updateLocalMap(main_object_modified_map_, map_in_);
    RCLCPP_INFO(this->get_logger(), "Static map updated.");

    // publish updated map
    nav_msgs::msg::OccupancyGrid updated_map;
    updated_map.header = map_in_->header;
    updated_map.info = map_in_->info;
    updated_map.data = map_in_->data;
    updated_map_publisher_->publish(updated_map);
    RCLCPP_INFO(this->get_logger(), "Updated map published.");
  }
}

// Calculate the counter values for each object
void DepthCameraSubscriber::calculateObjectTypeCount()
{
  nav_msgs::msg::OccupancyGrid modified_map;
  modified_map = objectDetection(cloud_transformed_member_);

  int min_object_pixels = 10;
  bool has_object = hasObject(modified_map, min_object_pixels);

  if (has_object)
  {
    // Calculate the centroid of the object in the map frame
    geometry_msgs::msg::Point object_centroid_in_map_frame_image_coordinate = calculateCentroidInMapFrame(modified_map);
    geometry_msgs::msg::Point object_centroid_in_map_frame_ros_coordinate = transformImgCoordinateToRos(map_in_->info.origin.position.x, map_in_->info.origin.position.y, map_in_->info.resolution,
                                                                                                        map_in_->info.height, object_centroid_in_map_frame_image_coordinate.x, object_centroid_in_map_frame_image_coordinate.y);
    geometry_msgs::msg::Transform map_to_robot_base_transform = findMapToRobotBaseTransformation(transform_2);
    // printMapToRobotBaseTransform(map_to_robot_base_transform_);
    geometry_msgs::msg::Point object_centroid_in_robot_base_frame = transformCentroidPointToRobotBase(object_centroid_in_map_frame_ros_coordinate, map_to_robot_base_transform);
    RCLCPP_INFO(
        this->get_logger(),
        "New object centroid in robot base frame is (%.2f, %.2f, %.2f)",
        object_centroid_in_robot_base_frame.x,
        object_centroid_in_robot_base_frame.y,
        object_centroid_in_robot_base_frame.z);

    // Check if this is the first time the object is seen
    if (timer_stamp_.seconds() == 0)
    {
      // Save the centroid in ros_coor and update the timer
      count_.push_back(1);
      RCLCPP_INFO(this->get_logger(), "1 added to counter");
      first_time_object_seen_centroid_ros_ = object_centroid_in_map_frame_ros_coordinate;
      main_object_modified_map_ = modified_map;
      timer_stamp_ = this->now();
      RCLCPP_INFO(this->get_logger(), "First time object is seen. Timer started.");
    }
    else
    {
      geometry_msgs::msg::Transform map_to_robot_base_transform = findMapToRobotBaseTransformation(transform_2);
      // printMapToRobotBaseTransform(map_to_robot_base_transform);

      geometry_msgs::msg::Point c1_object_centroid_in_robot_base_frame;
      c1_object_centroid_in_robot_base_frame = transformCentroidPointToRobotBase(first_time_object_seen_centroid_ros_, map_to_robot_base_transform);
      RCLCPP_INFO(
          this->get_logger(),
          "Main object centroid in robot base frame is (%.2f, %.2f, %.2f)",
          c1_object_centroid_in_robot_base_frame.x,
          c1_object_centroid_in_robot_base_frame.y,
          c1_object_centroid_in_robot_base_frame.z);

      geometry_msgs::msg::Point base_origin_coordinate; // Base_link origin coordinate
      base_origin_coordinate.x = 0.0;
      base_origin_coordinate.y = 0.0;
      base_origin_coordinate.z = 0.0;

      double distance = calculateDistance(c1_object_centroid_in_robot_base_frame, base_origin_coordinate);
      RCLCPP_INFO(
          this->get_logger(),
          "Distance from main object: %.2f",
          std::abs(distance));

      double angle = calculateAngle(c1_object_centroid_in_robot_base_frame);
      RCLCPP_INFO(
          this->get_logger(),
          "Angle for main object: %.2f",
          std::abs(angle));

      // Check the condition whether the main object is visible
      if (std::abs(distance) < 2.0 && std::abs(angle) < (87.0 / 2.0))
      {
        RCLCPP_INFO(this->get_logger(), "Main object is visible");
        double centroids_differences = calculateDistance(c1_object_centroid_in_robot_base_frame, object_centroid_in_robot_base_frame); // Calculate the distance between the centroids
        RCLCPP_INFO(
            this->get_logger(),
            "Distance differences from objects centroids: %.2f",
            centroids_differences);

        // Check if the centroids are close enough
        double distance_threshold = 0.1; // 10 cm
        if (centroids_differences <= distance_threshold)
        {

          // Check if the main object is seen from different robot POV
          modified_map_buffer_.push_back(main_object_modified_map_);
          modified_map_buffer_.push_back(modified_map);

          if (modified_map_buffer_.size() >= 2)
          {
            const nav_msgs::msg::OccupancyGrid &modified_map_1 = modified_map_buffer_[0];
            const nav_msgs::msg::OccupancyGrid &modified_map_2 = modified_map_buffer_[1];

            double percentage_threshold = 80; // 80% should be common
            bool result = compareOccupancyGridsFromDifferentRobotPOV(modified_map_1, modified_map_2, percentage_threshold);
            if (result)
            {
              RCLCPP_INFO(this->get_logger(), "The object is same from other POV.");
              RCLCPP_INFO(this->get_logger(), "---------------------------------------------");

              rclcpp::Duration time_difference = this->now() - timer_stamp_;
              if (time_difference.seconds() >= 60)
              {
                int intervals = static_cast<int>(time_difference.seconds()) / 60;
                for (int i = 0; i < intervals; ++i)
                {
                  count_.push_back(1);
                  RCLCPP_INFO(this->get_logger(), "1 added to counter");
                }
                timer_stamp_ = this->now();

                modified_map_buffer_.clear();
              }
              else
              {
                RCLCPP_INFO(this->get_logger(), "Object seen again within 60 seconds. Ignoring.");
              }
            }
          }
        }
        else
        {
          determineObjectType(count_);
          count_.clear();
          timer_stamp_ = rclcpp::Time(0, 0); // Reset the timer_stamp
        }
        if (count_.size() >= 8)
        {
          RCLCPP_INFO(
              this->get_logger(),
              "Object continuously visible for more than 8 unit time (2 hours). Running handle function.");
          determineObjectType(count_);
          count_.clear();
          timer_stamp_ = rclcpp::Time(0, 0);
        }
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Main object is not visible.");
        RCLCPP_INFO(this->get_logger(), "----------------------------------------------------------------------------");
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr DepthCameraSubscriber::updateLocalMap(const nav_msgs::msg::OccupancyGrid &dynamic_object_map, const nav_msgs::msg::OccupancyGrid::SharedPtr &static_map)
{
  nav_msgs::msg::OccupancyGrid::SharedPtr updated_static_map = static_map;

  for (int i = 0; i < dynamic_object_map.info.width * dynamic_object_map.info.height; ++i)
  {
    if (dynamic_object_map.data[i] == 100)
    {
      updated_static_map->data[i] = 100;
    }
  }
  return updated_static_map;
}
