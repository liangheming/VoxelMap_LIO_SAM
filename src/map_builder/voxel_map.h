#pragma once

#include <vector>
#include <memory>
#include <cstdint>
#include <Eigen/Eigen>
#include <unordered_map>
#define HASH_P 116101
#define MAX_N 10000000000

namespace lio
{

    class VoxelKey
    {
    public:
        int64_t x, y, z;
        VoxelKey(int64_t _x = 0, int64_t _y = 0, int64_t _z = 0) : x(_x), y(_y), z(_z) {}
        bool operator==(const VoxelKey &other) const
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
        Eigen::Vector3d eigens;
        Eigen::Matrix<double, 6, 6> plane_cov;
        bool is_valid;
        int points_size;
        double radius;
        double d;
    };

    class OctoTree
    {
    public:
        double quater_length;
        Eigen::Vector3d center;
        OctoTree() = default;
        OctoTree(int max_layer, int layer, std::vector<int> update_size_threshes, int max_point_thresh, double plane_thresh);
        void push_back(const PointWithCov &pv);
        void insert_back(const PointWithCov &pv);
        void init_plane(const std::vector<PointWithCov> &points);
        void update_plane(const std::vector<PointWithCov> &points);
        void initial_tree();
        void split_tree();
        int subIndex(const PointWithCov &pv, int *xyz);

    private:
        Plane plane_;
        int layer_;
        int max_layer_;
        bool is_leave_;
        bool is_initialized_;
        bool update_enable_;

        std::vector<std::shared_ptr<OctoTree>> leaves_;
        std::vector<PointWithCov> temp_points_;
        std::vector<PointWithCov> new_points_;
        int update_size_thresh_;
        std::vector<int> update_size_threshes_;
        double plane_thresh_;
        int max_point_thresh_;
        int update_size_thresh_for_new_;
        int all_point_num_;
        int new_point_num_;
    };

    class VoxelMap
    {
        typedef std::unordered_map<VoxelKey, std::shared_ptr<OctoTree>, VoxelKey::Hasher> FeatMap;

    public:
        FeatMap feat_map;
        void buildMap(const std::vector<PointWithCov> &input_points);
        void updateMap(const std::vector<PointWithCov> &input_points);
        VoxelMap(double voxel_size, int max_layer, std::vector<int> &update_size_threshes, int max_point_thresh, double plane_thresh)
            : voxel_size_(voxel_size), max_layer_(max_layer), update_size_threshes_(update_size_threshes), max_point_thresh_(max_point_thresh), plane_thresh_(plane_thresh)
        {
        }
        size_t size() { return feat_map.size(); }

    private:
        double voxel_size_;
        int max_layer_;
        std::vector<int> update_size_threshes_;
        int max_point_thresh_;
        double plane_thresh_;
    };
} // namespace lio
