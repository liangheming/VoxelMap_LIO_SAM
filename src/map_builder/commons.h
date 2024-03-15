#pragma once
#include <Eigen/Core>

namespace lio
{
    Eigen::Matrix3d skew(const Eigen::Vector3d &vec);

    Eigen::Matrix3d Exp(const Eigen::Vector3d &vec);

    Eigen::Vector3d Log(const Eigen::Matrix3d &mat);

    Eigen::Matrix3d Jr(const Eigen::Vector3d &vec);

    Eigen::Matrix3d Jr_inv(const Eigen::Vector3d &vec);
    

}
