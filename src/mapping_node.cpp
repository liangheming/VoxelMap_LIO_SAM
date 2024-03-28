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
#include <pcl/common/transforms.h>
#include <chrono>
#include "voxel_lio_sam/SaveMap.h"
#include "voxel_lio_sam/ConvertMap.h"

struct NodeParams
{
    std::string imu_topic;
    std::string lidar_topic;
    double lio_expect_hz = 50;
    int filter_num = 3;
    double range_min = 0.5;
    double range_max = 30.0;
};

struct NodeGroupData
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
        initServices();
        initSubScribers();
        initPublishers();
    }

    void initNodeParams()
    {
        nh_.param<std::string>("lidar_topic", params_.lidar_topic, "/livox/lidar");
        nh_.param<std::string>("imu_topic", params_.imu_topic, "/livox/imu");
        nh_.param<double>("lio_expect_hz", params_.lio_expect_hz, 50);
        nh_.param<int>("filter_num", params_.filter_num, 3);
        nh_.param<double>("range_min", params_.range_min, 0.5);
        nh_.param<double>("range_max", params_.range_max, 30.0);
        lio::LIOParams lio_params;
        nh_.param<double>("scan_resolution", lio_params.scan_resolution, 0.2);
        nh_.param<bool>("gravity_align", lio_params.gravity_align, true);
        nh_.param<int>("imu_init_num", lio_params.imu_init_num, 20);
        nh_.param<double>("na", lio_params.na, 0.01);
        nh_.param<double>("ng", lio_params.ng, 0.01);
        nh_.param<double>("nbg", lio_params.nbg, 0.0001);
        nh_.param<double>("nba", lio_params.nba, 0.0001);
        nh_.param<int>("opti_max_iter", lio_params.opti_max_iter, 5);
        std::vector<double> r_il, p_il;
        nh_.param<std::vector<double>>("r_il", r_il, std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
        assert(r_il.size() == 9);
        lio_params.r_il << r_il[0], r_il[1], r_il[2], r_il[3], r_il[4], r_il[5], r_il[6], r_il[7], r_il[8];
        nh_.param<std::vector<double>>("p_il", p_il, std::vector<double>{0, 0, 0});
        assert(p_il.size() == 3);
        lio_params.p_il << p_il[0], p_il[1], p_il[2];
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

    void initServices()
    {
        save_map_srv_ = nh_.advertiseService("save_map", &LioMappingNode::saveMapCB, this);
        convert_map_srv_ = nh_.advertiseService("convert_map", &LioMappingNode::convertMapCB, this);
    }

    void imuCB(const sensor_msgs::Imu::ConstPtr msg)
    {
        std::lock_guard<std::mutex> lock(group_data_.imu_mutex);
        double timestamp = msg->header.stamp.toSec();
        if (timestamp < group_data_.last_imu_time)
        {
            ROS_WARN("IMU TIME SYNC ERROR");
            group_data_.imu_buffer.clear();
        }
        group_data_.last_imu_time = timestamp;
        group_data_.imu_buffer.emplace_back(Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                                            Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                            timestamp);
    }

    void lidarCB(const livox_ros_driver2::CustomMsg::ConstPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        livox2pcl(msg, cloud, params_.filter_num, params_.range_min, params_.range_max);
        std::lock_guard<std::mutex> lock(group_data_.lidar_mutex);
        double timestamp = msg->header.stamp.toSec();
        if (timestamp < group_data_.last_lidar_time)
        {
            ROS_WARN("LIDAR TIME SYNC ERROR");
            group_data_.lidar_buffer.clear();
        }
        group_data_.last_lidar_time = timestamp;
        group_data_.lidar_buffer.emplace_back(timestamp, cloud);
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr publishBodyCloud(const double &time)
    {

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr body_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = current_state_.rot_ext.cast<float>();
        transform.block<3, 1>(0, 3) = current_state_.pos_ext.cast<float>();
        pcl::transformPointCloud(*sync_pack_.cloud, *body_cloud, transform);
        if (body_cloud_pub_.getNumSubscribers() >= 0)
            body_cloud_pub_.publish(pcl2msg(body_cloud, "body", time));

        return body_cloud;
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr publishWorldCloud(const double &time)
    {

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = (current_state_.rot * current_state_.rot_ext).cast<float>();
        transform.block<3, 1>(0, 3) = (current_state_.rot * current_state_.pos_ext + current_state_.pos).cast<float>();
        pcl::transformPointCloud(*sync_pack_.cloud, *world_cloud, transform);
        if (world_cloud_pub_.getNumSubscribers() >= 1)
            world_cloud_pub_.publish(pcl2msg(world_cloud, "map", time));
        return world_cloud;
    }

    void publishOdom(const double &time)
    {
        if (odom_pub_.getNumSubscribers() == 0)
            return;
        odom_pub_.publish(eigen2Odometry(current_state_.rot, current_state_.pos, "map", "body", time));
    }

    bool syncPackage()
    {
        if (group_data_.imu_buffer.empty() || group_data_.lidar_buffer.empty())
            return false;
        // 同步点云数据
        if (!group_data_.lidar_pushed)
        {
            sync_pack_.cloud = group_data_.lidar_buffer.front().second;
            sync_pack_.cloud_start_time = group_data_.lidar_buffer.front().first;
            sync_pack_.cloud_end_time = sync_pack_.cloud_start_time + sync_pack_.cloud->points.back().curvature / double(1000.0);
            group_data_.lidar_pushed = true;
        }
        // 等待IMU的数据
        if (group_data_.last_imu_time < sync_pack_.cloud_end_time)
            return false;

        sync_pack_.imus.clear();

        // 同步IMU的数据
        // IMU的最后一帧数据的时间小于点云最后一个点的时间
        while (!group_data_.imu_buffer.empty() && (group_data_.imu_buffer.front().timestamp < sync_pack_.cloud_end_time))
        {
            sync_pack_.imus.push_back(group_data_.imu_buffer.front());
            group_data_.imu_buffer.pop_front();
        }
        group_data_.lidar_buffer.pop_front();
        group_data_.lidar_pushed = false;
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
            // auto time_start = std::chrono::high_resolution_clock::now();

            map_builder_(sync_pack_);
            // auto time_end = std::chrono::high_resolution_clock::now();
            // double duration = std::chrono::duration_cast<std::chrono::duration<double>>(time_end - time_start).count() * 1000;
            // ROS_INFO("PROCESS DUTATION: %.4f", duration);
            if (map_builder_.currentStatus() != lio::LIOStatus::LIO_MAPPING)
                continue;
            current_state_ = map_builder_.currentState();
            br_.sendTransform(eigen2Transform(current_state_.rot, current_state_.pos, "map", "body", sync_pack_.cloud_end_time));
            publishBodyCloud(sync_pack_.cloud_end_time);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr wpc = publishWorldCloud(sync_pack_.cloud_end_time);
            clouds_.push_back(wpc);
            publishOdom(sync_pack_.cloud_end_time);
        }
    }

    bool saveMapCB(voxel_lio_sam::SaveMap::Request &req, voxel_lio_sam::SaveMap::Response &res)
    {
        std::string file_path = req.save_path;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        if (clouds_.size() == 0)
        {
            res.status = false;
            res.message = "Empty Cloud!";
            return true;
        }
        for (auto &pc : clouds_)
            *cloud += *pc;
        if (req.resolution > 0)
        {
            ROS_INFO("DOWN SAMPLE CLOUD");
            pcl::VoxelGrid<pcl::PointXYZINormal> filter;
            filter.setLeafSize(req.resolution, req.resolution, req.resolution);
            filter.setInputCloud(cloud);
            filter.filter(*cloud);
        }
        pcl::PCDWriter writer;
        writer.writeBinaryCompressed(file_path, *cloud);
        res.status = true;
        res.message = "save path: " + file_path;
        return true;
    }

    bool convertMapCB(voxel_lio_sam::ConvertMap::Request &req, voxel_lio_sam::ConvertMap::Response &res)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PCDReader reader;
        reader.read(req.source, *cloud);

        pcl::VoxelGrid<pcl::PointXYZINormal> filter;
        filter.setLeafSize(req.resolution, req.resolution, req.resolution);
        filter.setInputCloud(cloud);
        filter.filter(*cloud);

        pcl::PCDWriter writer;
        writer.writeBinaryCompressed(req.target, *cloud);
        res.status = true;
        res.message = "save path: " + req.target;
        return true;
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
    NodeGroupData group_data_;
    lio::SyncPackage sync_pack_;
    lio::LioBuilder map_builder_;
    kf::State current_state_;
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clouds_;
    ros::ServiceServer save_map_srv_;
    ros::ServiceServer convert_map_srv_;
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