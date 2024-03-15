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
};

struct NodeState
{
    double last_imu_time = 0.0;
    double last_lidar_time = 0.0;
    std::mutex lidar_mutex;
    std::mutex imu_mutex;
    std::deque<lio::IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
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

    void mainCB(const ros::TimerEvent &time_event)
    {
        ROS_INFO("%ld, %ld", state_.lidar_buffer.size(), state_.imu_buffer.size());
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