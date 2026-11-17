#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <memory>
#include <limits>
#include <vector>
#include <cstdio>
#include <unistd.h>
#include "../src/runtime_motion_primitives.h"

namespace {
using sensor_occupancy::Cell;
using sensor_occupancy::Point;
using sensor_occupancy::Ray;

class RuntimeSensorMapping : public ::testing::Test {
protected:
    std::unique_ptr<ros::NodeHandle> nh;
    std::string ns;
    void SetUp() override {
        static unsigned serial=0;
        ns="/runtime_sensor_map_test/case_"+std::to_string(++serial);
        nh.reset(new ros::NodeHandle(ns));
        nh->setParam("sensor_map/resolution",1.0);
        nh->setParam("sensor_map/clearance",0.0);
        nh->setParam("sensor_map/max_age",1.0);
        nh->setParam("sensor_map/frame","test_lidar");
        nh->setParam("sensor_map/sensor_offset",std::vector<double>{0,0,0});
        nh->setParam("sensor_map/enabled",true);
        nh->setParam("recovery/enabled",true);
        nh->setParam("recovery/require_terminal",true);
        nh->setParam("recovery/strict_execution",true);
        nh->setParam("recovery/state_only_features",true);
        nh->setParam("motion_primitive/horizon_time",1.0);
        nh->setParam("recovery/horizon_time",1.0);
        nh->setParam("motion_primitive/max_altitude",100.0);
        nh->setParam("waypoints/wp1/x",100.0);
        nh->setParam("waypoints/wp1/y",.5);
        nh->setParam("waypoints/wp1/h",20.5);
    }
    void TearDown() override {nh.reset();ros::param::del(ns);}
    static ros::Time stamp(double value) {ros::Time t;t.fromSec(value);return t;}
    nav_msgs::Odometry pose(double time,double x=.5,double y=.5,double z=10.5,double yaw=0) const {
        nav_msgs::Odometry m;m.header.stamp=stamp(time);m.header.frame_id="world";
        m.pose.pose.position.x=x;m.pose.pose.position.y=y;m.pose.pose.position.z=z;
        m.pose.pose.orientation=tf::createQuaternionMsgFromYaw(yaw);
        m.twist.twist.linear.x=15.5;m.twist.twist.angular.x=1;
        return m;
    }
    sensor_msgs::PointCloud2 cloud(double time,const std::vector<Ray>& rays,
                                   const std::string& frame="test_lidar",bool organized=false) const {
        sensor_msgs::PointCloud2 m;m.header.stamp=stamp(time);m.header.frame_id=frame;
        m.height=1;m.is_dense=false;
        sensor_msgs::PointCloud2Modifier modifier(m);
        modifier.setPointCloud2Fields(4,"x",1,sensor_msgs::PointField::FLOAT32,
            "y",1,sensor_msgs::PointField::FLOAT32,"z",1,sensor_msgs::PointField::FLOAT32,
            "hit",1,sensor_msgs::PointField::FLOAT32);
        modifier.resize(rays.size());
        sensor_msgs::PointCloud2Iterator<float> x(m,"x"),y(m,"y"),z(m,"z"),hit(m,"hit");
        for(const auto& ray:rays){*x=ray.endpoint.x;*y=ray.endpoint.y;*z=ray.endpoint.z;*hit=ray.hit;++x;++y;++z;++hit;}
        if(organized&&rays.size()==4){m.width=2;m.height=2;m.row_step=2*m.point_step;}
        return m;
    }
    static void pump(double seconds=.04) {
        const auto end=ros::WallTime::now()+ros::WallDuration(seconds);
        while(ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.002).sleep();}
    }
    ros::Publisher publisher() {
        auto pub=nh->advertise<sensor_msgs::PointCloud2>("lidar/rays",2);
        const auto end=ros::WallTime::now()+ros::WallDuration(2);
        while(pub.getNumSubscribers()==0&&ros::WallTime::now()<end){ros::spinOnce();ros::WallDuration(.005).sleep();}
        EXPECT_GT(pub.getNumSubscribers(),0u);
        return pub;
    }
    RuntimeMotionPrimitives::s_t trim(const RuntimeMotionPrimitives& planner) const {
        RuntimeMotionPrimitives::s_t state;state.V=15.5;state.h=20.5;state.pos[0]=state.pos[1]=.5;
        double lo=-20*Rad2Deg,hi=25*Rad2Deg;
        for(int i=0;i<60;++i){const double a=(lo+hi)/2;
            const double residual=1.86*state.V*state.V*(planner.C_D(a)*std::tan(a)+planner.C_L(a))-9.81;
            if(residual>0)hi=a;else lo=a;}
        state.attack=(lo+hi)/2;
        state.T=1.86*state.V*state.V*planner.C_D(state.attack)/std::cos(state.attack);
        return state;
    }
};

TEST_F(RuntimeSensorMapping, OrganizedRaysCreateFreeHitAndUnknownCells) {
    RuntimeSensorMap mapper(*nh);mapper.add_pose(pose(1));auto pub=publisher();
    const double nan=std::numeric_limits<double>::quiet_NaN();
    pub.publish(cloud(1,{Ray(Point(4,0,0),1),Ray(Point(0,4,0),0),
                         Ray(Point(1,1,0),-1),Ray(Point(nan,0,0),1)},"/test_lidar",true));
    pump();
    EXPECT_TRUE(mapper.ready(1));
    EXPECT_EQ(mapper.map().state(Point(3.5,.5,10.5)),Cell::FREE);
    EXPECT_EQ(mapper.map().state(Point(4.5,.5,10.5)),Cell::OCCUPIED);
    EXPECT_EQ(mapper.map().state(Point(5.5,.5,10.5)),Cell::UNKNOWN);
    EXPECT_EQ(mapper.map().state(Point(.5,4.5,10.5)),Cell::FREE);
    EXPECT_EQ(mapper.map().state(Point(.5,5.5,10.5)),Cell::UNKNOWN);
    EXPECT_EQ(mapper.map().stats().valid_rays,2u);
    EXPECT_EQ(mapper.map().stats().occupied_cells,1u);
}

TEST_F(RuntimeSensorMapping, CloudWaitsForPoseBracketAndUsesInterpolatedPosition) {
    RuntimeSensorMap mapper(*nh);mapper.add_pose(pose(1,.5));auto pub=publisher();
    pub.publish(cloud(1.05,{Ray(Point(4,0,0),1)}));pump();
    EXPECT_FALSE(mapper.ready(1.05));EXPECT_EQ(mapper.map().size(),0u);
    mapper.add_pose(pose(1.1,2.5));
    EXPECT_TRUE(mapper.ready(1.05));
    EXPECT_EQ(mapper.map().state(Point(5.5,.5,10.5)),Cell::OCCUPIED);
    EXPECT_EQ(mapper.map().state(Point(4.5,.5,10.5)),Cell::FREE);
    EXPECT_EQ(mapper.map().state(Point(6.5,.5,10.5)),Cell::UNKNOWN);
}

TEST_F(RuntimeSensorMapping, RotationInterpolationAlsoRotatesSensorOffset) {
    nh->setParam("sensor_map/sensor_offset",std::vector<double>{1,0,0});
    RuntimeSensorMap mapper(*nh);
    mapper.add_pose(pose(1,.5,.5,10.5,0));
    mapper.add_pose(pose(1.1,.5,.5,10.5,M_PI/2));auto pub=publisher();
    pub.publish(cloud(1.05,{Ray(Point(4,0,0),1)}));pump();
    const double end=.5+5/std::sqrt(2.0);
    EXPECT_EQ(mapper.map().state(Point(end,end,10.5)),Cell::OCCUPIED);
    EXPECT_EQ(mapper.map().state(Point(5.5,.5,10.5)),Cell::UNKNOWN);
}

TEST_F(RuntimeSensorMapping, WrongFrameAndOverwidePoseGapDoNotPublishReadyMap) {
    RuntimeSensorMap mapper(*nh);mapper.add_pose(pose(1));auto pub=publisher();
    pub.publish(cloud(1,{Ray(Point(4,0,0),1)},"wrong_sensor"));pump();
    EXPECT_FALSE(mapper.ready(1));EXPECT_EQ(mapper.map().size(),0u);
    pub.publish(cloud(1.1,{Ray(Point(4,0,0),1)}));pump();
    mapper.add_pose(pose(1.2)); // Interpolation gap > 0.15s is rejected.
    EXPECT_FALSE(mapper.ready(1.2));EXPECT_EQ(mapper.map().size(),0u);
}

TEST_F(RuntimeSensorMapping, PoseClockResetClearsMapAndQueuedFutureScan) {
    RuntimeSensorMap mapper(*nh);mapper.add_pose(pose(2));auto pub=publisher();
    pub.publish(cloud(2,{Ray(Point(4,0,0),1)}));pump();ASSERT_TRUE(mapper.ready(2));
    pub.publish(cloud(2.1,{Ray(Point(8,0,0),1)}));pump();
    mapper.add_pose(pose(1,50.5));
    EXPECT_FALSE(mapper.ready(1));EXPECT_EQ(mapper.map().size(),0u);
    mapper.add_pose(pose(2.1,50.5)); // Old future scan must have been discarded.
    EXPECT_EQ(mapper.map().size(),0u);
    EXPECT_EQ(mapper.map().state(Point(4.5,.5,10.5)),Cell::UNKNOWN);
}

TEST_F(RuntimeSensorMapping, ReadinessExpiresAndRejectsFutureEpoch) {
    RuntimeSensorMap mapper(*nh);mapper.add_pose(pose(1));auto pub=publisher();
    pub.publish(cloud(1,{Ray(Point(4,0,0),0)}));pump();
    EXPECT_TRUE(mapper.ready(1.999));EXPECT_FALSE(mapper.ready(2.001));
    EXPECT_FALSE(mapper.ready(.9));
}

TEST_F(RuntimeSensorMapping, InvalidExtrinsicsAndResolutionAreRejected) {
    nh->setParam("sensor_map/resolution",0.0);
    EXPECT_THROW({RuntimeSensorMap mapper(*nh);},std::invalid_argument);
    nh->setParam("sensor_map/resolution",1.0);
    nh->setParam("sensor_map/sensor_offset",std::vector<double>{0,std::numeric_limits<double>::infinity(),0});
    EXPECT_THROW({RuntimeSensorMap mapper(*nh);},std::invalid_argument);
}

TEST_F(RuntimeSensorMapping, SensorRuntimeRequiresStrictGateAndRejectsTruthGeometry) {
    nh->setParam("recovery/strict_execution",false);
    EXPECT_THROW({RuntimeMotionPrimitives planner(*nh);},std::invalid_argument);
    nh->setParam("recovery/strict_execution",true);
    nh->setParam("box_obstacles/num",1);
    for(const std::string& axis:{"x","y","z"})nh->setParam("box_obstacles/box1/"+axis,10.0);
    for(const std::string& axis:{"x","y","z"})nh->setParam("box_obstacles/box1/size_"+axis,2.0);
    EXPECT_THROW({RuntimeMotionPrimitives planner(*nh);},std::invalid_argument);
    nh->setParam("box_obstacles/num",0);
    RuntimeMotionPrimitives planner(*nh);
    RuntimeMotionPrimitives::box_obstacle_t box;box.geometry={10,10,10,2,2,2};
    EXPECT_THROW(planner.set_box_obstacles({box}),std::invalid_argument);
}

TEST_F(RuntimeSensorMapping, NoControlIsPublishedBeforeAnySensorMapExists) {
    RuntimeMotionPrimitives planner(*nh);
    unsigned commands=0;std::string status;
    auto command_sub=nh->subscribe<geometry_msgs::Twist>("cmd_vel",10,[&](const geometry_msgs::Twist::ConstPtr&){++commands;});
    auto status_sub=nh->subscribe<std_msgs::String>("planning_status",10,[&](const std_msgs::String::ConstPtr& m){status=m->data;});
    pump(.06);
    planner.odom_cb(pose(ros::Time::now().toSec(),.5,.5,20.5));
    planner.planningCallback(ros::TimerEvent());pump(.04);
    EXPECT_EQ(commands,0u);
    EXPECT_NE(status.find("\"status\":\"MAP_WAITING\""),std::string::npos);
    EXPECT_NE(status.find("\"command_published\":false"),std::string::npos);
    EXPECT_NE(status.find("\"map_ready\":false"),std::string::npos);
    EXPECT_NE(status.find("\"truth_obstacle_count\":0"),std::string::npos);
}

TEST_F(RuntimeSensorMapping, UnknownDelayBridgeCannotBeTreatedAsFree) {
    RuntimeMotionPrimitives planner(*nh);
    RuntimeMotionPrimitives::s_t observed,predicted;
    observed.V=15.5;observed.h=20.5;observed.pos[0]=.5;observed.pos[1]=.5;
    EXPECT_FALSE(planner.predict_state_at(observed,stamp(1),stamp(1.1),predicted));
}

TEST_F(RuntimeSensorMapping, AcquisitionSnapshotRoundTripsCountsAndMeasuredEpoch) {
    char path[]="/tmp/runtime_sensor_map_snapshot_XXXXXX";
    const int fd=mkstemp(path);ASSERT_GE(fd,0);close(fd);
    nh->setParam("sensor_map/snapshot_path",std::string(path));
    {
        RuntimeSensorMap mapper(*nh);mapper.add_pose(pose(1));auto pub=publisher();
        pub.publish(cloud(1,{Ray(Point(4,0,0),1)}));pump();ASSERT_TRUE(mapper.ready(1));
    }
    std::ifstream input(path,std::ios::binary);ASSERT_TRUE(input.good());
    char magic[8];double resolution,epoch;std::uint64_t count[2];
    input.read(magic,8);input.read(reinterpret_cast<char*>(&resolution),8);
    input.read(reinterpret_cast<char*>(&epoch),8);input.read(reinterpret_cast<char*>(count),16);
    EXPECT_EQ(std::string(magic,8),"SENMAP01");EXPECT_DOUBLE_EQ(resolution,1);
    EXPECT_DOUBLE_EQ(epoch,1);EXPECT_GT(count[0],0u);EXPECT_EQ(count[1],1u);
    input.seekg(0,std::ios::end);EXPECT_EQ(static_cast<std::uint64_t>(input.tellg()),40+12*(count[0]+count[1]));
    input.close();std::remove(path);
}

TEST_F(RuntimeSensorMapping, InvalidPlanningReserveAndLookaheadAreRejected) {
    for(double value:{-1.0,std::numeric_limits<double>::infinity()}) {
        nh->setParam("sensor_map/planning_reserve",value);
        EXPECT_THROW({RuntimeMotionPrimitives planner(*nh);},std::invalid_argument);
    }
    nh->setParam("sensor_map/planning_reserve",0.0);
    for(double value:{-.01,.151,std::numeric_limits<double>::infinity()}) {
        nh->setParam("execution/planning_lookahead",value);
        EXPECT_THROW({RuntimeMotionPrimitives planner(*nh);},std::invalid_argument);
    }
}

TEST_F(RuntimeSensorMapping, NominalReserveCannotEraseMinimumClearanceCachedWitness) {
    nh->setParam("sensor_map/clearance",.25);
    nh->setParam("sensor_map/planning_reserve",1.0);
    nh->setParam("motion_primitive/vertical_num",1);
    nh->setParam("motion_primitive/state_relative_altitudes",false);
    nh->setParam("recovery/max_candidates",29);
    RuntimeMotionPrimitives planner(*nh);const auto state=trim(planner);
    auto observation=pose(1,.5,.5,20.5);
    planner.odom_cb(observation);auto pub=publisher();
    // Actual ray evidence fills a two-metre-high corridor. It can admit the
    // minimum .25m witness but cannot contain the larger 1.25m nominal cube.
    std::vector<Ray> rays;
    for(int x=-3;x<=40;++x)for(int y=-5;y<=5;++y)for(int z=20;z<=21;++z)
        rays.emplace_back(Point(x,y,z-20),0);
    pub.publish(cloud(1,rays));pump(.1);
    EXPECT_DOUBLE_EQ(planner.sensor_reserve_at(0),0);
    EXPECT_NEAR(planner.sensor_reserve_at(.2f),0,1e-7);
    EXPECT_NEAR(planner.sensor_reserve_at(.35f),.5,1e-6);
    EXPECT_DOUBLE_EQ(planner.sensor_reserve_at(1),1);
    const auto minimum=planner.select_motion_primitive(state);
    ASSERT_TRUE(minimum.recovery_verified);
    planner.accept_selection(minimum);const float duration=planner.remaining_execution_time();
    ASSERT_GT(duration,0);
    auto nominal=planner.select_planning_proposal(state);
    EXPECT_FALSE(nominal.recovery_verified);
    EXPECT_FALSE(nominal.reused_verified_plan);
    EXPECT_FLOAT_EQ(planner.remaining_execution_time(),duration);
    EXPECT_FALSE(planner.rollout_motion_primitive(state,minimum.command).collision);
    ASSERT_TRUE(planner.prepare_execution(state,nominal));
    EXPECT_EQ(nominal.status,"REVALIDATED_PLAN");
    EXPECT_TRUE(nominal.reused_verified_plan);
    EXPECT_FLOAT_EQ(planner.remaining_execution_time(),duration);
}
} // namespace

int main(int argc,char** argv){ros::init(argc,argv,"runtime_sensor_map_test");::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
