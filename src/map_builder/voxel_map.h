#pragma once

#include <vector>
#include <memory>
#include <cstdint>
#include <Eigen/Eigen>
#define HASH_P 116101
#define MAX_N 10000000000

namespace lio
{

    class VoxelKey
    {
    public:
        int64_t x, y, z;
        VoxelKey(int64_t _x = 0, int64_t _y = 0, int64_t _z = 0) : x(_x), y(_y), z(_z) {}
        bool operator==(const VoxelKey &other)
        {
            return (x == other.x && y == other.y && z == other.z);
        }
        struct Hasher
        {
            int64_t operator()(const VoxelKey &k) const
            {
                return ((((k.z) * HASH_P) % MAX_N + (k.y)) * HASH_P) % MAX_N + (k.x);
            }
        };
    };
    struct PointWithCov
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d point;
        Eigen::Matrix3d cov;
    };
    struct Plane
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d center;
        Eigen::Vector3d normal;
        Eigen::Matrix3d covariance;
        Eigen::Matrix<double, 6, 6> plane_cov;
        bool is_valid;
        int points_size;
    };
    class OctoTree
    {
    public:
        OctoTree() = default;
        OctoTree(int max_layer, int layer, std::vector<int> update_size_threshes, int max_point_thresh, double plane_thresh);
        void push_back(const PointWithCov& pv);
        void insert_back(const PointWithCov& pv);
        void init_plane(const std::vector<PointWithCov> &points);
        void update_plane(const std::vector<PointWithCov> &points);

    private:
        Plane plane_;
        int layer_;
        int max_layer_;
        bool is_leave_;
        bool is_initialized_;
        bool update_enable_;
        Eigen::Vector3d center_;
        std::vector<std::shared_ptr<OctoTree>> leaves_;
        std::vector<PointWithCov> temp_points_;
        std::vector<PointWithCov> new_points_;
        int update_size_thresh_;
        std::vector<int> update_size_threshes_;
        double plane_thresh_;
        int max_point_thresh_;
        int update_size_thresh_for_new_;
        double quater_length_;
        int all_point_num_;
        int new_point_num_;
    };
} // namespace lio
