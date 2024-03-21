#include "voxel_map.h"
#include <iostream>
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
        plane_.is_valid = false;
        leaves_.resize(8, nullptr);
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

        Eigen::Matrix3d J_Q = Eigen::Matrix3d::Identity() * 1.0 / static_cast<double>(plane_.points_size);
        plane_.eigens << evalsReal(evalsMin), evalsReal(evalsMid), evalsReal(evalsMax);
        plane_.normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);

        plane_.radius = std::sqrt(plane_.eigens(2));

        if (plane_.eigens(0) < plane_thresh_)
        {
            std::vector<int> index(points.size());
            std::vector<Eigen::Matrix<double, 6, 6>> temp_matrix(points.size());
            for (int i = 0; i < points.size(); i++)
            {
                Eigen::Matrix<double, 6, 3> J;
                Eigen::Matrix3d F;
                for (int m = 0; m < 3; m++)
                {
                    if (m != (int)evalsMin)
                    {
                        Eigen::Matrix<double, 1, 3> F_m =
                            (points[i].point - plane_.center).transpose() /
                            ((plane_.points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
                            (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                             evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
                        F.row(m) = F_m;
                    }
                    else
                    {
                        Eigen::Matrix<double, 1, 3> F_m;
                        F_m << 0, 0, 0;
                        F.row(m) = F_m;
                    }
                }
                J.block<3, 3>(0, 0) = evecs.real() * F;
                J.block<3, 3>(3, 0) = J_Q;
                plane_.plane_cov += J * points[i].cov * J.transpose();
            }
            plane_.is_valid = true;
        }
        else
        {
            plane_.is_valid = false;
        }
    }

    void OctoTree::update_plane(const std::vector<PointWithCov> &points)
    {
        Eigen::Matrix3d old_covariance = plane_.covariance;
        Eigen::Vector3d old_center = plane_.center;
        Eigen::Matrix3d sum_ppt =
            (plane_.covariance + plane_.center * plane_.center.transpose()) *
            plane_.points_size;
        Eigen::Vector3d sum_p = plane_.center * plane_.points_size;
        for (size_t i = 0; i < points.size(); i++)
        {
            Eigen::Vector3d pv = points[i].point;
            sum_ppt += pv * pv.transpose();
            sum_p += pv;
        }
        plane_.points_size = plane_.points_size + points.size();
        plane_.center = sum_p / plane_.points_size;
        plane_.covariance = sum_ppt / plane_.points_size - plane_.center * plane_.center.transpose();
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

        plane_.eigens << evalsReal(evalsMin), evalsReal(evalsMid), evalsReal(evalsMax);
        plane_.normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane_.radius = std::sqrt(plane_.eigens(2));
        // plane_.d = plane_.center.dot(plane_.normal);
        if (plane_.eigens(0) < plane_thresh_)
        {
            plane_.is_valid = true;
        }
        else
        {
            plane_.is_valid = false;
        }
    }

    void OctoTree::initial_tree()
    {
        if (temp_points_.size() < update_size_thresh_)
            return;
        is_initialized_ = true;
        new_point_num_ = 0;
        init_plane(temp_points_);
        if (plane_.is_valid)
        {
            is_leave_ = true;
            if (temp_points_.size() > max_point_thresh_)
            {
                update_enable_ = false;
                std::vector<PointWithCov>().swap(temp_points_);
            }
            else
            {
                update_enable_ = true;
            }
        }
        else
        {
            is_leave_ = false;
            split_tree();
        }
    }

    void OctoTree::split_tree()
    {
        if (layer_ >= max_layer_)
        {
            is_leave_ = true;
            return;
        }

        for (size_t i = 0; i < temp_points_.size(); i++)
        {
            int xyz[3] = {0, 0, 0};
            int leafnum = subIndex(temp_points_[i], xyz);
            if (leaves_[leafnum] == nullptr)
            {
                leaves_[leafnum] = std::make_shared<OctoTree>(max_layer_, layer_ + 1, update_size_threshes_, max_point_thresh_, plane_thresh_);
                Eigen::Vector3d shift((2 * xyz[0] - 1) * quater_length, (2 * xyz[1] - 1) * quater_length, (2 * xyz[2] - 1) * quater_length);
                leaves_[leafnum]->center = center + shift;
                leaves_[leafnum]->quater_length = quater_length / 2;
            }
            leaves_[leafnum]->push_back(temp_points_[i]);
        }

        for (uint i = 0; i < 8; i++)
        {
            if (leaves_[i] != nullptr)
            {
                leaves_[i]->initial_tree();
            }
        }
    }

    int OctoTree::subIndex(const PointWithCov &pv, int *xyz)
    {
        if (pv.point[0] > center[0])
            xyz[0] = 1;
        if (pv.point[1] > center[1])
            xyz[1] = 1;
        if (pv.point[2] > center[2])
            xyz[2] = 1;
        return 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    }

    void OctoTree::insert_back(const PointWithCov &pv)
    {
        if (!is_initialized_)
        {
            push_back(pv);
            new_point_num_++;
            initial_tree();
        }
        else
        {
            if (plane_.is_valid)
            {
                if (update_enable_)
                {
                    push_back(pv);
                    new_point_num_++;
                    if (new_point_num_ > update_size_thresh_for_new_)
                    {
                        init_plane(temp_points_);
                        new_point_num_ = 0;
                    }

                    if (all_point_num_ >= max_point_thresh_)
                    {
                        update_enable_ = false;
                        std::vector<PointWithCov>().swap(temp_points_);
                    }
                }
                else
                {
                    // TODO: norm_compare
                }
            }
            else
            {
                if (layer_ < max_layer_)
                {
                    if (temp_points_.size() != 0)
                        std::vector<PointWithCov>().swap(temp_points_);
                    int xyz[3] = {0, 0, 0};
                    int leafnum = subIndex(pv, xyz);
                    if (leaves_[leafnum] == nullptr)
                    {
                        leaves_[leafnum] = std::make_shared<OctoTree>(max_layer_, layer_ + 1, update_size_threshes_, max_point_thresh_, plane_thresh_);
                        Eigen::Vector3d shift((2 * xyz[0] - 1) * quater_length, (2 * xyz[1] - 1) * quater_length, (2 * xyz[2] - 1) * quater_length);
                        leaves_[leafnum]->center = center + shift;
                        leaves_[leafnum]->quater_length = quater_length / 2;
                    }
                    leaves_[leafnum]->insert_back(pv);
                }
                else
                {
                    if (update_enable_)
                    {
                        push_back(pv);
                        new_point_num_++;
                    }
                    if (new_point_num_ > update_size_thresh_for_new_)
                    {
                        init_plane(temp_points_);
                        new_point_num_ = 0;
                    }

                    if (all_point_num_ >= max_point_thresh_)
                    {
                        update_enable_ = false;
                        std::vector<PointWithCov>().swap(temp_points_);
                    }
                }
            }
        }
    }

    void VoxelMap::buildMap(const std::vector<PointWithCov> &input_points)
    {
        uint plsize = input_points.size();
        for (uint i = 0; i < plsize; i++)
        {
            const PointWithCov &p_v = input_points[i];
            Eigen::Vector3d idx = (p_v.point / voxel_size_).array().floor();
            VoxelKey k(static_cast<int64_t>(idx(0)), static_cast<int64_t>(idx(1)), static_cast<int64_t>(idx(2)));
            auto iter = feat_map.find(k);
            if (iter == feat_map.end())
            {
                feat_map[k] = std::make_shared<OctoTree>(max_layer_, 0, update_size_threshes_, max_point_thresh_, plane_thresh_);
                feat_map[k]->center = Eigen::Vector3d((0.5 + k.x) * voxel_size_, (0.5 + k.y) * voxel_size_, (0.5 + k.z) * voxel_size_);
                feat_map[k]->quater_length = voxel_size_ / 4;
            }
            feat_map[k]->push_back(p_v);
        }
        for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter)
        {
            iter->second->initial_tree();
        }
    }

    void VoxelMap::updateMap(const std::vector<PointWithCov> &input_points)
    {
        uint plsize = input_points.size();
        for (uint i = 0; i < plsize; i++)
        {
            const PointWithCov p_v = input_points[i];
            Eigen::Vector3d idx = (p_v.point / voxel_size_).array().floor();
            VoxelKey k(static_cast<int64_t>(idx(0)), static_cast<int64_t>(idx(1)), static_cast<int64_t>(idx(2)));
            auto iter = feat_map.find(k);
            if (iter == feat_map.end())
            {
                feat_map[k] = std::make_shared<OctoTree>(max_layer_, 0, update_size_threshes_, max_point_thresh_, plane_thresh_);
                feat_map[k]->center = Eigen::Vector3d((0.5 + k.x) * voxel_size_, (0.5 + k.y) * voxel_size_, (0.5 + k.z) * voxel_size_);
                feat_map[k]->quater_length = voxel_size_ / 4;
            }
            feat_map[k]->insert_back(p_v);
        }
    }

    void VoxelMap::buildResidual(ResidualData &info, std::shared_ptr<OctoTree> oct_tree)
    {
        info.is_valid = false;
        if (oct_tree->plane().is_valid)
        {
            Eigen::Vector3d p_world_to_center = info.point_world - oct_tree->plane().center;
            info.plane_center = oct_tree->plane().center;
            info.plane_norm = oct_tree->plane().normal;
            info.plane_cov = oct_tree->plane().plane_cov;
            info.residual = info.plane_norm.transpose() * p_world_to_center;
            double dis_to_plane = std::abs(info.residual);
            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = p_world_to_center;
            J_nq.block<1, 3>(0, 3) = -info.plane_norm;
            double sigma_l = J_nq * info.plane_cov * J_nq.transpose();
            sigma_l += info.plane_norm.transpose() * info.cov * info.plane_norm;
            if (dis_to_plane < info.sigma_num * sqrt(sigma_l))
            {
                info.is_valid = true;
            }
        }
        else
        {
            if (info.current_layer < max_layer_)
            {
                for (size_t i = 0; i < 8; i++)
                {
                    if (oct_tree->leaves()[i] == nullptr)
                        continue;
                    info.current_layer += 1;
                    buildResidual(info, oct_tree->leaves()[i]);
                    if (info.is_valid)
                        break;
                }
            }
        }
    }
} // namespace lio
