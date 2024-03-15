#include "lio_builder.h"

namespace lio
{
    LioBuidler::LioBuidler(LioParams &params) : params_(params)
    {
        kf_.setMaxIter(params.max_iter);
        kf_.set_share_function([this](State &s, SharedState &d)
                               { computeResidual(s, d); });
        status_ = Status::INIT;
        Q_.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d::Identity() * params_.gyro_cov;
        Q_.block<3, 3>(3, 3).diagonal() = Eigen::Vector3d::Identity() * params_.acc_cov;
        Q_.block<3, 3>(6, 6).diagonal() = Eigen::Vector3d::Identity() * params_.gyro_bias_cov;
        Q_.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d::Identity() * params_.acc_bias_cov;
    }

} // namespace lio
