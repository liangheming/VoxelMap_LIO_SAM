#pragma once
#include "ieskf.h"

namespace lio
{
    enum Status
    {
        INIT,
        MAPPING
    };
    struct LioParams
    {
        int max_iter = 5;
        double acc_cov = 0.01;
        double gyro_cov = 0.01;
        double acc_bias_cov = 0.0001;
        double gyro_bias_cov = 0.0001;
        Eigen::Matrix3d imu_ext_rot = Eigen::Matrix3d::Identity();
        Eigen::Vector3d imu_ext_pos = Eigen::Vector3d::Zero();
    };
    class LioBuidler
    {
    public:
        LioBuidler(LioParams &params);

        void computeResidual(State &state, SharedState &share_state);

        void processIMU();

        void processLidar();

    private:
        IESKF kf_;
        Status status_;
        LioParams params_;

        IMUData last_imu_;
        Matrix12d Q_;
    };
} // namespace lio
