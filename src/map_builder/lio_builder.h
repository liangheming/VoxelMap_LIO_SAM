#pragma once
#include "ieskf.h"
#include <queue>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

namespace lio
{
    enum Status
    {
        INIT,
        MAPPING
    };
    struct LioParams
    {
        int max_iter = 5;
        double acc_cov = 0.01;
        double gyro_cov = 0.01;
        double acc_bias_cov = 0.0001;
        double gyro_bias_cov = 0.0001;
        bool align_gravity = true;
        int init_imu_num = 20;
        Eigen::Matrix3d imu_ext_rot = Eigen::Matrix3d::Identity();
        Eigen::Vector3d imu_ext_pos = Eigen::Vector3d::Zero();
    };
    class LioBuidler
    {
    public:
        LioBuidler();

        LioBuidler(LioParams &params);

        void init(LioParams &params);

        void computeResidual(State &state, SharedState &share_state);

        void processIMU(const IMUData &imu);

        void processLidar(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, double lidar_start_time);

        State currentState() { return kf_.x(); }

    private:
        IESKF kf_;
        Status status_;
        LioParams params_;

        IMUData last_imu_;
        Matrix12d Q_;
        bool first_lidar_;
        std::deque<IMUData> imus_;
        std::vector<Pose> imu_poses_;
        Eigen::Vector3d last_acc_;
        Eigen::Vector3d last_gyro_;
        double init_gravity_norm_;
        double last_lidar_end_time_;
    };
} // namespace lio
