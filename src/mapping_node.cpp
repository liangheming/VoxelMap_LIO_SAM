#include <mutex>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/lio_builder.h"

struct NodeParams
{
    std::string imu_topic;
    std::string lidar_topic;
    double lio_expect_hz = 50;
    int filter_num = 3;
    double blind_range = 0.5;
};

struct NodeState
{
    double last_imu_time = 0.0;
    double last_lidar_time = 0.0;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    std::deque<lio::IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    bool lidar_pushed = false;
};

class LioMappingNode
{

public:
    LioMappingNode(ros::NodeHandle &nh, tf2_ros::TransformBroadcaster &br) : nh_(nh), br_(br)
    {
        initNodeParams();
        initSubScribers();
        initPublishers();
    }

    void initNodeParams()
    {
        nh_.param<std::string>("lidar_topic", params_.lidar_topic, "/livox/lidar");
        nh_.param<std::string>("imu_topic", params_.imu_topic, "/livox/imu");
        nh_.param<double>("lio_expect_hz", params_.lio_expect_hz, 50);
        nh_.param<int>("filter_num", params_.filter_num, 3);
        nh_.param<double>("blind_range", params_.blind_range, 0.5);

        lio::LIOParams lio_params;
        map_builder_.initialize(lio_params);
    }

    void initSubScribers()
    {
        lidar_sub_ = nh_.subscribe(params_.lidar_topic, 10000, &LioMappingNode::lidarCB, this);
        imu_sub_ = nh_.subscribe(params_.imu_topic, 10000, &LioMappingNode::imuCB, this);
    }

    void initPublishers()
    {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("slam_odom", 1000);
        body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("body_cloud", 1000);
        world_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("world_cloud", 1000);
    }

    void imuCB(const sensor_msgs::Imu::ConstPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_.imu_mutex);
        double timestamp = msg->header.stamp.toSec();
        if (timestamp < state_.last_imu_time)
        {
            ROS_WARN("IMU TIME SYNC ERROR");
            state_.imu_buffer.clear();
        }
        state_.last_imu_time = timestamp;
        state_.imu_buffer.emplace_back(Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                                       Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                       timestamp);
    }

    void lidarCB(const livox_ros_driver2::CustomMsg::ConstPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        livox2pcl(msg, cloud, params_.filter_num, params_.blind_range);
        std::lock_guard<std::mutex> lock(state_.lidar_mutex);
        double timestamp = msg->header.stamp.toSec();
        if (timestamp < state_.last_lidar_time)
        {
            ROS_WARN("LIDAR TIME SYNC ERROR");
            state_.lidar_buffer.clear();
        }
        state_.last_lidar_time = timestamp;
        state_.lidar_buffer.emplace_back(timestamp, cloud);
    }

    void publishBodyCloud(const double &time)
    {
        if (body_cloud_pub_.getNumSubscribers() == 0)
            return;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr body_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        body_cloud->resize(sync_pack_.cloud->size());

        for (int i = 0; i < sync_pack_.cloud->size(); i++)
        {
            pcl::PointXYZINormal &p = sync_pack_.cloud->points[i];
            Eigen::Vector3d p_b(p.x, p.y, p.z);
            Eigen::Vector3d p_w = current_state_.rot_ext * p_b + current_state_.pos_ext;
            body_cloud->points[i].x = p_w(0);
            body_cloud->points[i].y = p_w(1);
            body_cloud->points[i].z = p_w(2);
            body_cloud->points[i].intensity = p.intensity;
        }
        body_cloud_pub_.publish(pcl2msg(body_cloud, "body", time));
    }

    void publishWorldCloud(const double &time)
    {
        if (world_cloud_pub_.getNumSubscribers() == 0)
            return;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        world_cloud->resize(sync_pack_.cloud->size());
        for (int i = 0; i < sync_pack_.cloud->size(); i++)
        {
            pcl::PointXYZINormal &p = sync_pack_.cloud->points[i];
            Eigen::Vector3d p_b(p.x, p.y, p.z);
            Eigen::Vector3d p_w = current_state_.rot * (current_state_.rot_ext * p_b + current_state_.pos_ext) + current_state_.pos;
            world_cloud->points[i].x = p_w(0);
            world_cloud->points[i].y = p_w(1);
            world_cloud->points[i].z = p_w(2);
            world_cloud->points[i].intensity = p.intensity;
        }
        world_cloud_pub_.publish(pcl2msg(world_cloud, "map", time));
    }

    void publishOdom(const double &time)
    {
        if (odom_pub_.getNumSubscribers() == 0)
            return;
        odom_pub_.publish(eigen2Odometry(current_state_.rot, current_state_.pos, "map", "body", time));
    }

    bool syncPackage()
    {
        if (state_.imu_buffer.empty() || state_.lidar_buffer.empty())
            return false;
        // 同步点云数据
        if (!state_.lidar_pushed)
        {
            sync_pack_.cloud = state_.lidar_buffer.front().second;
            sync_pack_.cloud_start_time = state_.lidar_buffer.front().first;
            sync_pack_.cloud_end_time = sync_pack_.cloud_start_time + sync_pack_.cloud->points.back().curvature / double(1000.0);
            state_.lidar_pushed = true;
        }
        // 等待IMU的数据
        if (state_.last_imu_time < sync_pack_.cloud_end_time)
            return false;

        sync_pack_.imus.clear();

        // 同步IMU的数据
        // IMU的最后一帧数据的时间小于点云最后一个点的时间
        while (!state_.imu_buffer.empty() && (state_.imu_buffer.front().timestamp < sync_pack_.cloud_end_time))
        {
            sync_pack_.imus.push_back(state_.imu_buffer.front());
            state_.imu_buffer.pop_front();
        }
        state_.lidar_buffer.pop_front();
        state_.lidar_pushed = false;
        return true;
    }

    void run()
    {
        ros::Rate rate(params_.lio_expect_hz);
        while (ros::ok())
        {
            rate.sleep();
            ros::spinOnce();
            if (!syncPackage())
                continue;
            map_builder_(sync_pack_);
            if (map_builder_.currentStatus() != lio::LIOStatus::LIO_MAPPING)
                continue;
            current_state_ = map_builder_.currentState();
            br_.sendTransform(eigen2Transform(current_state_.rot, current_state_.pos, "map", "body", sync_pack_.cloud_end_time));
            publishBodyCloud(sync_pack_.cloud_end_time);
            publishWorldCloud(sync_pack_.cloud_end_time);
            publishOdom(sync_pack_.cloud_end_time);
        }
    }

private:
    ros::NodeHandle &nh_;
    tf2_ros::TransformBroadcaster &br_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;

    ros::Publisher odom_pub_;
    ros::Publisher body_cloud_pub_;
    ros::Publisher world_cloud_pub_;

    ros::Timer main_timer_;
    NodeParams params_;
    NodeState state_;
    lio::SyncPackage sync_pack_;
    lio::LioBuilder map_builder_;
    kf::State current_state_;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh("~");
    tf2_ros::TransformBroadcaster br;
    LioMappingNode mapping_node(nh, br);
    mapping_node.run();
    return 0;
}