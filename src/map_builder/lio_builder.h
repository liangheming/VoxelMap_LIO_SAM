#pragma once
#include <queue>
#include "ieskf.h"
#include "commons.h"
#include <Eigen/Eigen>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include "voxel_map.h"

namespace lio
{
    enum LIOStatus
    {
        IMU_INIT,
        MAP_INIT,
        LIO_MAPPING
    };
    struct LIOParams
    {
        int opti_max_iter = 5;
        double na = 0.01;
        double ng = 0.01;
        double nba = 0.0001;
        double nbg = 0.0001;
        bool gravity_align = true;
        bool estimate_ext = false;

        int imu_init_num = 20;
        Eigen::Matrix3d r_il = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p_il = Eigen::Vector3d::Zero();
        double scan_resolution = 0.25;
        double map_resolution = 0.25;

        int max_layer = 2;
        double voxel_size = 0.5;
        std::vector<int> update_size_threshes = std::vector<int>{20, 10};
        int max_point_thresh = 200;
        double plane_thresh = 0.01;

        double ranging_cov = 0.04;
        double angle_cov = 0.1;
    };

    struct LIODataGroup
    {
        IMUData last_imu;
        std::vector<IMUData> imu_cache;
        std::vector<Pose> imu_poses_cache;
        Eigen::Vector3d last_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_gyro = Eigen::Vector3d::Zero();
        double last_cloud_end_time = 0.0;
        double gravity_norm;
        kf::Matrix12d Q = kf::Matrix12d::Identity();

        std::vector<ResidualData> residual_info;
    };

    class LioBuilder
    {
    public:
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_lidar;

        LioBuilder() = default;

        void initialize(LIOParams &params);

        void operator()(SyncPackage &package);

        bool initializeImu(std::vector<IMUData> &imus);

        void undistortCloud(SyncPackage &package);

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformToWorld(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);

        kf::State currentState() { return kf_.x(); }

        LIOStatus currentStatus() { return status_; }

        void sharedUpdateFunc(kf::State &state, kf::SharedState &shared_state);

    private:
        kf::IESKF kf_;
        LIOParams params_;
        LIOStatus status_;
        LIODataGroup data_group_;
        pcl::VoxelGrid<pcl::PointXYZINormal> scan_filter_;
        std::shared_ptr<VoxelMap> map_;
    };
} // namespace lio
