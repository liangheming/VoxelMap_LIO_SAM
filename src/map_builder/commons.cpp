#include "commons.h"
namespace lio
{
    Eigen::Matrix3d skew(const Eigen::Vector3d &vec)
    {
        Eigen::Matrix3d ret;
        ret << 0.0, -vec(2), vec(1),
            vec(2), 0.0, -vec(0),
            -vec(1), vec(0), 0.0;
        return ret;
    }

    Eigen::Matrix3d Exp(const Eigen::Vector3d &vec)
    {
        double norm = vec.norm();
        Eigen::Matrix3d skew_vec = skew(vec);
        if (norm < 1e-4)
            return Eigen::Matrix3d::Identity() + skew_vec;
        return Eigen::Matrix3d::Identity() + std::sin(norm) / norm * skew_vec + (1 - std::cos(norm)) / (norm * norm) * skew_vec * skew_vec;
    }

    Eigen::Vector3d Log(const Eigen::Matrix3d &mat)
    {
        double phi = std::acos((mat.trace() - 1) / 2);
        if (std::abs(phi) < 1e-4)
            return Eigen::Vector3d::Zero();
        Eigen::Matrix3d skew_mat = phi * (mat - mat.transpose()) / (2 * std::sin(phi));
        return Eigen::Vector3d(skew_mat(2, 1), skew_mat(0, 2), skew_mat(1, 0));
    }

    Eigen::Matrix3d Jr(const Eigen::Vector3d &vec)
    {
        double norm = vec.norm();
        Eigen::Matrix3d skew_vec = skew(vec);
        if (norm < 1e-4)
            return Eigen::Matrix3d::Identity() + 0.5 * skew_vec;
        return Eigen::Matrix3d::Identity() - (1 - std::cos(norm)) / (norm * norm) * skew_vec + (norm - std::sin(norm)) / (norm * norm * norm) * skew_vec * skew_vec;
    }

    Eigen::Matrix3d Jr_inv(const Eigen::Vector3d &vec)
    {
        double norm = vec.norm();
        Eigen::Matrix3d skew_vec = skew(vec);
        if (norm < 1e-4)
            return Eigen::Matrix3d::Identity() + 0.5 * skew_vec;
        return Eigen::Matrix3d::Identity() + 0.5 * skew_vec + (1 / (norm * norm) + (1 + std::cos(norm)) / (2 * norm * std::sin(norm))) * skew_vec * skew_vec;
    }

}
