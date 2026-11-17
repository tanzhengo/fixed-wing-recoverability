// Exact multi-line ray-to-PointCloud2 bridge for the simulated aircraft lidar.
// Ray endpoints use the same angular indices as Gazebo's collision rays.
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include "lidar_ray_encoding.h"

namespace gazebo {
class GazeboRos3DLidar : public SensorPlugin {
 public:
  void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override {
    sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(parent);
    if (!sensor_) {
      gzerr << "GazeboRos3DLidar requires a CPU ray sensor\n";
      return;
    }
    if (!ros::isInitialized()) {
      gzerr << "GazeboRos3DLidar requires gazebo_ros_api_plugin\n";
      return;
    }
    if (sensor_->RayCount() < 2 || sensor_->VerticalRayCount() < 2 ||
        sensor_->RayCount() != sensor_->RangeCount() ||
        sensor_->VerticalRayCount() != sensor_->VerticalRangeCount()) {
      gzerr << "3D lidar requires at least two rays on each axis and scan resolution 1\n";
      return;
    }
    const auto value = [&](const std::string& name, const std::string& fallback) {
      return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
    };
    frame_ = value("frameName", "suav1/lidar");
    while (!frame_.empty() && frame_.front() == '/') frame_.erase(0, 1);
    node_.reset(new ros::NodeHandle(value("robotNamespace", "/suav1")));
    publisher_ = node_->advertise<sensor_msgs::PointCloud2>(
        value("topicName", "lidar/points_raw"), 2);
    ray_publisher_ = node_->advertise<sensor_msgs::PointCloud2>(
        value("rayTopicName", "lidar/rays"), 2);
    // The shape's NewLaserScans event fires before RaySensor updates its time.
    // Sensor::ConnectUpdated runs after ranges and measurement time are ready.
    update_connection_ = sensor_->ConnectUpdated(std::bind(&GazeboRos3DLidar::OnScan, this));
    sensor_->SetActive(true);
    ROS_INFO_STREAM("3D lidar: " << sensor_->RayCount() << " x "
                    << sensor_->VerticalRayCount() << ", frame " << frame_
                    << ", hits " << publisher_.getTopic()
                    << ", rays " << ray_publisher_.getTopic());
  }

 protected:
  void OnScan() {
    if (!node_) return;
    const bool publish_hits = publisher_.getNumSubscribers() != 0;
    const bool publish_rays = ray_publisher_.getNumSubscribers() != 0;
    if (!publish_hits && !publish_rays) return;
    const unsigned horizontal = sensor_->RayCount();
    const unsigned vertical = sensor_->VerticalRayCount();
    const double yaw_min = sensor_->AngleMin().Radian();
    const double yaw_step = (sensor_->AngleMax().Radian() - yaw_min) / (horizontal - 1);
    const double pitch_min = sensor_->VerticalAngleMin().Radian();
    const double pitch_step = (sensor_->VerticalAngleMax().Radian() - pitch_min) / (vertical - 1);
    const double near = sensor_->RangeMin(), far = sensor_->RangeMax();
    struct Point { float x, y, z, intensity; };
    std::vector<Point> hits;
    std::vector<lidar_ray_encoding::Endpoint> rays;
    if (publish_hits) hits.reserve(horizontal * vertical);
    if (publish_rays) rays.reserve(horizontal * vertical);
    for (unsigned j = 0; j < vertical; ++j) {
      const double pitch = pitch_min + j * pitch_step;
      for (unsigned i = 0; i < horizontal; ++i) {
        const unsigned index = j * horizontal + i;
        const double range = sensor_->LaserShape()->GetRange(index);
        const double yaw = yaw_min + i * yaw_step;
        const auto endpoint = lidar_ray_encoding::endpoint(range, near, far, yaw, pitch);
        if (publish_rays) rays.push_back(endpoint);
        // Preserve the old hit-only stream: no-return rays must never become
        // a shell of occupied voxels in its visualization consumer.
        if (publish_hits && endpoint.hit == 1.0f) {
          const double intensity = sensor_->LaserShape()->GetRetro(index);
          hits.push_back({endpoint.x, endpoint.y, endpoint.z,
                         static_cast<float>(std::isfinite(intensity) ? intensity : 0)});
        }
      }
    }
    // RaySensor has already set this time for the current ranges. Do not query
    // LastUpdateTime here: Sensor holds its update-time mutex during this event.
    const auto stamp = sensor_->LastMeasurementTime();
    const ros::Time scan_time(stamp.sec, stamp.nsec);
    if (publish_hits) {
      sensor_msgs::PointCloud2 cloud;
      cloud.header.stamp = scan_time;
      cloud.header.frame_id = frame_;
      cloud.height = 1;
      cloud.is_dense = true;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
          "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32,
          "intensity", 1, sensor_msgs::PointField::FLOAT32);
      modifier.resize(hits.size());
      if (!hits.empty()) {
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"),
            z(cloud, "z"), intensity(cloud, "intensity");
        for (const auto& point : hits) {
          *x = point.x; *y = point.y; *z = point.z; *intensity = point.intensity;
          ++x; ++y; ++z; ++intensity;
        }
      }
      publisher_.publish(cloud);
    }
    if (publish_rays) {
      sensor_msgs::PointCloud2 cloud;
      cloud.header.stamp = scan_time;
      cloud.header.frame_id = frame_;
      cloud.height = 1;
      cloud.is_dense = false;  // Invalid rays retain their organized position.
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
          "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32,
          "hit", 1, sensor_msgs::PointField::FLOAT32);
      modifier.resize(rays.size());
      cloud.height = vertical;
      cloud.width = horizontal;
      cloud.row_step = cloud.point_step * horizontal;
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"),
          z(cloud, "z"), hit(cloud, "hit");
      for (const auto& point : rays) {
        *x = point.x; *y = point.y; *z = point.z; *hit = point.hit;
        ++x; ++y; ++z; ++hit;
      }
      ray_publisher_.publish(cloud);
    }
  }

 private:
  sensors::RaySensorPtr sensor_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Publisher publisher_, ray_publisher_;
  std::string frame_;
  event::ConnectionPtr update_connection_;
};
GZ_REGISTER_SENSOR_PLUGIN(GazeboRos3DLidar)
}  // namespace gazebo
