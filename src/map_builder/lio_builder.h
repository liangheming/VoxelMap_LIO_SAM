#pragma once
#include "ieskf.h"
#include <queue>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include "ikd_Tree.h"

namespace lio
{
    float sq_dist(const pcl::PointXYZINormal &p1, const pcl::PointXYZINormal &p2);

    bool esti_plane(Eigen::Vector4d &out, const KD_TREE<pcl::PointXYZINormal>::PointVector &points, const double &thresh);

    enum Status
    {
        INIT,
        MAPPING
    };
    struct LocalMap
    {
        double cube_len = 500.0;
        double det_range = 100.0;
        double move_thresh = 1.5;
        bool is_initialed = false;
        BoxPointType local_map_corner;
        std::vector<BoxPointType> cub_to_rm;
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
        Eigen::Vector3d imu_ext_pos = Eigen::Vector3d(-0.011, -0.02329, 0.04412);
        double scan_resolution = 0.5;
        double map_resolution = 0.5;
    };
    class LioBuilder
    {
    public:
        LioBuilder();

        LioBuilder(LioParams &params);

        void init(LioParams &params);

        void computeResidual(State &state, SharedState &share_state);

        void processIMU(const IMUData &imu);

        void processLidar(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, double lidar_start_time);

        void undistortCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, double lidar_start_time);

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformToWorld(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);

        void increaseMap();

        void trimMap();

        State currentState() { return kf_.x(); }

    private:
        IESKF kf_;
        Status status_;
        LioParams params_;

        IMUData last_imu_;
        Matrix12d Q_;
        bool first_lidar_, map_init_;
        std::deque<IMUData> imus_;
        std::vector<Pose> imu_poses_;
        Eigen::Vector3d last_acc_;
        Eigen::Vector3d last_gyro_;
        double init_gravity_norm_;
        double last_lidar_end_time_;

        pcl::VoxelGrid<pcl::PointXYZINormal> scan_filter_;
        std::shared_ptr<KD_TREE<pcl::PointXYZINormal>> ikdtree_;
        LocalMap local_map_;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr effect_cloud_lidar_;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr effect_norm_vec_;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_down_lidar_;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_down_world_;
        std::vector<KD_TREE<pcl::PointXYZINormal>::PointVector> nearest_points_;
        std::vector<bool> point_selected_flag_;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr norm_vec_;
    };
} // namespace lio
