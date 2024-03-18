#include "voxel_map.h"
namespace lio
{

    OctoTree::OctoTree(int max_layer, int layer, std::vector<int> update_size_threshes, int max_point_thresh, double plane_thresh)
        : max_layer_(max_layer),
          layer_(layer),
          update_size_threshes_(update_size_threshes),
          max_point_thresh_(max_point_thresh),
          plane_thresh_(plane_thresh)
    {
        temp_points_.clear();
        new_points_.clear();
        is_leave_ = false;
        all_point_num_ = 0;
        new_point_num_ = 0;
        update_size_thresh_for_new_ = 5;
        is_initialized_ = false;
        update_enable_ = true;
        update_size_thresh_ = update_size_threshes_[layer_];
        leaves_.reserve(8);
        for (int i = 0; i < 8; i++)
            leaves_[i] = nullptr;
    }

    void OctoTree::push_back(const PointWithCov &pv)
    {
        temp_points_.push_back(pv);
        all_point_num_++;
    }
    void OctoTree::init_plane(const std::vector<PointWithCov> &points)
    {
        plane_.plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
        plane_.covariance = Eigen::Matrix3d::Zero();
        plane_.center = Eigen::Vector3d::Zero();
        plane_.normal = Eigen::Vector3d::Zero();
        plane_.points_size = points.size();

        for (auto pv : points)
        {
            plane_.covariance += pv.point * pv.point.transpose();
            plane_.center += pv.point;
        }
        plane_.center = plane_.center / plane_.points_size;
        plane_.covariance = plane_.covariance / plane_.points_size - plane_.center * plane_.center.transpose();
        Eigen::EigenSolver<Eigen::Matrix3d> es(plane_.covariance);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal = evals.real();

        Eigen::Matrix3d::Index evalsMin, evalsMax;
        evalsReal.rowwise().sum().minCoeff(&evalsMin);
        evalsReal.rowwise().sum().maxCoeff(&evalsMax);
        int evalsMid = 3 - evalsMin - evalsMax;

        Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
        Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
        Eigen::Vector3d evecMax = evecs.real().col(evalsMax);

        
    }
} // namespace lio
