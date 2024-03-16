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
        IMUData() = default;
        IMUData(const Eigen::Vector3d &a, const Eigen::Vector3d &g, double &d) : acc(a), gyro(g), sec(d) {}
    };

    Eigen::Vector3d rotate2rpy(Eigen::Matrix3d &rot);

    struct Pose
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        Eigen::Matrix3d rot;
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Pose();
        Pose(double t, Eigen::Vector3d a, Eigen::Vector3d g, Eigen::Vector3d v, Eigen::Vector3d p, Eigen::Matrix3d r)
            : offset(t), acc(a), gyro(g), vel(v), pos(p), rot(r) {}
        double offset;
    };

}
