#include <mutex>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include "utils.h"
#include <sensor_msgs/Imu.h>
#include "map_builder/commons.h"

struct RosParams
{
    std::string imu_topic;
    std::string lidar_topic;
    double lio_hz = 50;
    int filter_num = 3;
    double blind = 0.5;
};

struct NodeState
{
    double last_imu_time = 0.0;
    double last_lidar_time = 0.0;
    std::mutex lidar_mutex;
    std::mutex imu_mutex;
    std::deque<lio::IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    std::pair<pcl::PointCloud<pcl::PointXYZINormal>::Ptr, std::vector<lio::IMUData>> package;
    bool lidar_packed = false;
    double package_begin_time = 0.0;
    double package_end_time = 0.0;
};

class LioMappingNode
{
public:
    LioMappingNode(ros::NodeHandle &nh) : nh_(nh)
    {
        initParams();
        initSubScribers();
        main_timer_ = nh_.createTimer(ros::Duration(1 / params_.lio_hz), &LioMappingNode::mainCB, this);
    }

    void initParams()
    {
        nh_.param<std::string>("lidar_topic", params_.lidar_topic, "/livox/lidar");
        nh_.param<std::string>("imu_topic", params_.imu_topic, "/livox/imu");
        nh_.param<double>("lio_hz", params_.lio_hz, 50);
        nh_.param<int>("filter_num", params_.filter_num, 3);
        nh_.param<double>("blind", params_.blind, 0.5);

        state_.package.second.reserve(200);
    }

    void initSubScribers()
    {
        lidar_sub_ = nh_.subscribe(params_.lidar_topic, 100, &LioMappingNode::lidarCB, this);
        imu_sub_ = nh_.subscribe(params_.imu_topic, 100, &LioMappingNode::imuCB, this);
    }

    void lidarCB(const livox_ros_driver2::CustomMsg::ConstPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        livox2pcl(msg, cloud, 3, 0.5);
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

    bool syncPackage()
    {
        if (state_.imu_buffer.empty() || state_.lidar_buffer.empty())
            return false;
        if (!state_.lidar_packed)
        {
            state_.package.first = state_.lidar_buffer.front().second;
            state_.package_begin_time = state_.lidar_buffer.front().first;
            state_.package_end_time = state_.package_begin_time + state_.lidar_buffer.front().second->points.back().curvature / 1000.0;
            state_.lidar_packed = true;
        }
        // 等待IMU的数据
        if (state_.last_imu_time < state_.package_end_time)
            return false;

        state_.package.second.clear();

        while (!state_.imu_buffer.empty() && (state_.imu_buffer.front().sec < state_.package_end_time))
        {
            state_.package.second.push_back(state_.imu_buffer.front());
            state_.imu_buffer.pop_front();
        }

        state_.lidar_buffer.pop_front();
        state_.lidar_packed = false;

        return true;
    }
    void mainCB(const ros::TimerEvent &time_event)
    {
        if (!syncPackage())
            return;
        
        
    }

private:
    ros::NodeHandle &nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Timer main_timer_;
    RosParams params_;
    NodeState state_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh("~");
    LioMappingNode node(nh);
    ros::spin();
}