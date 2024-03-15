#pragma once
#include <Eigen/Eigen>

namespace lio
{
    Eigen::Matrix3d skew(const Eigen::Vector3d &vec);

    Eigen::Matrix3d Exp(const Eigen::Vector3d &vec);

    Eigen::Vector3d Log(const Eigen::Matrix3d &mat);

    Eigen::Matrix3d Jr(const Eigen::Vector3d &vec);

    Eigen::Matrix3d Jr_inv(const Eigen::Vector3d &vec);

    struct IMUData
    {
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        double sec;
        IMUData(const Eigen::Vector3d &a,const Eigen::Vector3d &g, double &d) : acc(a), gyro(g), sec(d) {}
    };
}
