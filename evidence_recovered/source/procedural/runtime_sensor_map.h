#ifndef RUNTIME_SENSOR_MAP_H
#define RUNTIME_SENSOR_MAP_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <deque>
#include <memory>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <cstdint>
#include "sensor_occupancy_map.h"

// Sensor-only map adapter. No scene geometry or Gazebo model services enter it.
class RuntimeSensorMap {
public:
    using Point = sensor_occupancy::Point;
    explicit RuntimeSensorMap(ros::NodeHandle node) : nh_(node) {
        nh_.param<double>("sensor_map/resolution", resolution_, 2.0);
        nh_.param<double>("sensor_map/max_age", max_age_, 1.0);
        nh_.param<double>("sensor_map/clearance", clearance_, 2.0);
        int capacity; nh_.param<int>("sensor_map/max_cells", capacity, 4000000);
        if (!std::isfinite(max_age_) || max_age_ <= 0 ||
            !std::isfinite(clearance_) || clearance_ < 0 || capacity < 1)
            throw std::invalid_argument("Invalid sensor map configuration");
        map_.reset(new sensor_occupancy::Map(resolution_, capacity));
        std::string topic; nh_.param<std::string>("sensor_map/ray_topic", topic, "lidar/rays");
        nh_.param<std::string>("sensor_map/frame", frame_, "suav1/lidar");
        nh_.param<std::string>("sensor_map/snapshot_path", snapshot_path_, "");
        if (!frame_.empty() && frame_.front() == '/') frame_.erase(frame_.begin());
        std::vector<double> offset;
        nh_.param<std::vector<double>>("sensor_map/sensor_offset", offset, {0.0, 0.0, 0.15});
        if (offset.size()!=3 || !std::isfinite(offset[0]) || !std::isfinite(offset[1]) || !std::isfinite(offset[2]))
            throw std::invalid_argument("Invalid sensor extrinsics");
        offset_ = tf::Vector3(offset[0],offset[1],offset[2]);
        ray_sub_ = nh_.subscribe(topic, 2, &RuntimeSensorMap::rays, this);
        status_pub_ = nh_.advertise<std_msgs::String>("sensor_map/status", 1, true);
    }
    ~RuntimeSensorMap() {
        if (snapshot_path_.empty()) return;
        try {
            const auto free=map_->snapshot(sensor_occupancy::Cell::FREE);
            const auto occupied=map_->snapshot(sensor_occupancy::Cell::OCCUPIED);
            std::ofstream out(snapshot_path_,std::ios::binary);
            if (!out) return;
            out.write("SENMAP01",8);
            out.write(reinterpret_cast<const char*>(&resolution_),sizeof(double));
            out.write(reinterpret_cast<const char*>(&last_stamp_),sizeof(double));
            const std::uint64_t counts[2]={free.size(),occupied.size()};
            out.write(reinterpret_cast<const char*>(counts),sizeof(counts));
            for (const auto* points : {&free,&occupied}) for (const auto& p:*points) {
                const float xyz[3]={static_cast<float>(p.x),static_cast<float>(p.y),static_cast<float>(p.z)};
                out.write(reinterpret_cast<const char*>(xyz),sizeof(xyz));
            }
        } catch (const std::exception&) { /* Optional acquisition export cannot throw on shutdown. */ }
    }
    void add_pose(const nav_msgs::Odometry& msg) {
        const double stamp=msg.header.stamp.toSec();
        const auto& p=msg.pose.pose.position; const auto& q=msg.pose.pose.orientation;
        tf::Quaternion rotation(q.x,q.y,q.z,q.w);
        if (stamp<=0 || !std::isfinite(stamp) || !std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(p.z) || !std::isfinite(rotation.length2()) || rotation.length2()<1e-8) return;
        if (!poses_.empty() && stamp<poses_.back().stamp) {
            poses_.clear(); pending_.clear(); map_->reset(); last_stamp_=0; ++time_resets_;
        }
        rotation.normalize();
        Pose pose{stamp,tf::Vector3(p.x,p.y,p.z),rotation};
        if (!poses_.empty() && stamp==poses_.back().stamp) poses_.back()=pose;
        else poses_.push_back(pose);
        while (poses_.size()>2 && stamp-poses_[1].stamp>4.0) poses_.pop_front();
        process();
    }
    bool ready(double now) const { return last_stamp_>0 && now>=last_stamp_-1e-6 && now-last_stamp_<=max_age_; }
    double stamp() const { return last_stamp_; }
    double clearance() const { return clearance_; }
    const sensor_occupancy::Map& map() const { return *map_; }
    bool free_segment(const Point& a,const Point& b,double extra_clearance=0) const {
        return std::isfinite(extra_clearance) && extra_clearance>=0 &&
            map_->segment_is_free(a,b,clearance_+extra_clearance);
    }
    bool free_point(double x,double y,double z,double extra_clearance=0) const {
        return free_segment({x,y,z},{x,y,z},extra_clearance);
    }
    float distance(double x,double y,double z) const {
        return static_cast<float>(map_->clearance_lower_bound({x,y,z},30.0));
    }
public:
    struct Pose { double stamp; tf::Vector3 position; tf::Quaternion rotation; };
    struct Scan { double stamp; std::vector<sensor_occupancy::Ray> rays; };
    bool pose_at(double stamp,Pose& pose) const {
        if (poses_.empty() || stamp<poses_.front().stamp || stamp>poses_.back().stamp) return false;
        for (size_t i=0;i<poses_.size();++i) {
            if (poses_[i].stamp==stamp) { pose=poses_[i];return true; }
            if (poses_[i].stamp>stamp && i>0) {
                const auto& a=poses_[i-1];const auto& b=poses_[i];
                if (b.stamp-a.stamp>.15) return false;
                const double f=(stamp-a.stamp)/(b.stamp-a.stamp);
                pose={stamp,a.position.lerp(b.position,f),a.rotation.slerp(b.rotation,f)};return true;
            }
        }
        return false;
    }
    void rays(const sensor_msgs::PointCloud2ConstPtr& msg) {
        std::string frame=msg->header.frame_id;
        if (!frame.empty() && frame.front()=='/') frame.erase(frame.begin());
        if (frame!=frame_) { ++dropped_;return; }
        Scan scan;scan.stamp=msg->header.stamp.toSec();
        if (!std::isfinite(scan.stamp) || scan.stamp<=last_stamp_) { ++dropped_;return; }
        try {
            sensor_msgs::PointCloud2ConstIterator<float> x(*msg,"x"),y(*msg,"y"),z(*msg,"z"),hit(*msg,"hit");
            scan.rays.reserve(msg->width*msg->height);
            for (;x!=x.end();++x,++y,++z,++hit) {
                ++input_beams_;
                if (!std::isfinite(*hit) || (*hit!=0.f && *hit!=1.f) ||
                    !std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
                scan.rays.push_back({{*x,*y,*z},static_cast<int>(*hit)});++valid_rays_;
            }
        } catch (const std::runtime_error&) { ++dropped_;return; }
        if (pending_.size()>=2) { pending_.pop_front();++dropped_; }
        pending_.push_back(std::move(scan));process();
    }
    void process() {
        while (!pending_.empty()) {
            const auto& scan=pending_.front();
            if (poses_.empty() || scan.stamp>poses_.back().stamp) break;
            Pose pose;
            if (scan.stamp<=last_stamp_ || !pose_at(scan.stamp,pose)) { pending_.pop_front();++dropped_;continue; }
            const auto origin=pose.position+tf::quatRotate(pose.rotation,offset_);
            std::vector<sensor_occupancy::Ray> world;world.reserve(scan.rays.size());
            for (const auto& ray:scan.rays) {
                const auto endpoint=origin+tf::quatRotate(pose.rotation,tf::Vector3(ray.endpoint.x,ray.endpoint.y,ray.endpoint.z));
                world.push_back({{endpoint.x(),endpoint.y(),endpoint.z()},ray.hit});
            }
            if (!map_->integrate_scan({origin.x(),origin.y(),origin.z()},world,scan.stamp)) {
                pending_.pop_front();++dropped_;continue;
            }
            last_stamp_=scan.stamp;pending_.pop_front();++integrated_;
            publish_status();
        }
    }
    void publish_status() {
        const auto s=map_->stats();
        std::ostringstream out;out<<std::setprecision(17)
            <<"{\"ready\":true,\"map_source\":\"lidar_rays\",\"last_stamp\":"<<last_stamp_
            <<",\"resolution\":"<<resolution_<<",\"clearance\":"<<clearance_
            <<",\"free_cells\":"<<s.free_cells<<",\"occupied_cells\":"<<s.occupied_cells
            <<",\"integrated_scans\":"<<integrated_<<",\"dropped_scans\":"<<dropped_
            <<",\"dropped_capacity\":"<<s.dropped_capacity
            <<",\"input_beams\":"<<input_beams_<<",\"valid_rays\":"<<valid_rays_
            <<",\"time_resets\":"<<time_resets_<<"}";
        std_msgs::String msg;msg.data=out.str();status_pub_.publish(msg);
    }
    ros::NodeHandle nh_;ros::Subscriber ray_sub_;ros::Publisher status_pub_;
    std::unique_ptr<sensor_occupancy::Map> map_;
    std::deque<Pose> poses_;std::deque<Scan> pending_;
    tf::Vector3 offset_;std::string frame_,snapshot_path_;
    double resolution_=2,clearance_=2,max_age_=1,last_stamp_=0;
    size_t integrated_=0,dropped_=0,input_beams_=0,valid_rays_=0,time_resets_=0;
};
#endif
