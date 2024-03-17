#pragma once
#include <queue>

#include "ieskf.h"
#include "commons.h"
#include <Eigen/Eigen>

#include "ikd_Tree.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

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
        int imu_init_num = 20;
        Eigen::Matrix3d r_il = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p_il = Eigen::Vector3d::Zero();
        double scan_resolution = 0.25;
        double map_resolution = 0.25;
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
    };

    struct LocalMap
    {
        double cube_len = 300;
        double det_range = 60;
        double move_thresh = 1.5;
        bool is_initialed = false;
        BoxPointType local_map_corner;
        std::vector<BoxPointType> cub_to_rm;
    };

    struct FASTLIODataGroup
    {
        double resolution = 0.5;
        std::shared_ptr<KD_TREE<pcl::PointXYZINormal>> ikdtree;
        LocalMap local_map;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_down_lidar;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_down_world;
        std::vector<KD_TREE<pcl::PointXYZINormal>::PointVector> nearest_points;
        std::vector<bool> point_selected_flag;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr norm_vec;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr effect_cloud_lidar;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr effect_norm_vec;

        FASTLIODataGroup() = default;
        void initialize(double _resolution);
        bool esti_plane(Eigen::Vector4d &out, const KD_TREE<pcl::PointXYZINormal>::PointVector &points, const double &thresh);
        void trimMap(kf::State state);
        void increaseMap(kf::State state);
        void sharedUpdateFunc(kf::State &state, kf::SharedState &share_data);
    };

    class LioBuilder
    {
    public:
        LioBuilder() = default;

        void initialize(LIOParams &params);

        void operator()(SyncPackage &package);

        bool initializeImu(std::vector<IMUData> &imus);

        void undistortCloud(SyncPackage &package);

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformToWorld(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);

        kf::State currentState() { return kf_.x(); }

        LIOStatus currentStatus() { return status_; }

    private:
        kf::IESKF kf_;
        LIOParams params_;
        LIOStatus status_;
        LIODataGroup data_group_;
        FASTLIODataGroup fastlio_data_;
        pcl::VoxelGrid<pcl::PointXYZINormal> scan_filter_;
    };
} // namespace lio
