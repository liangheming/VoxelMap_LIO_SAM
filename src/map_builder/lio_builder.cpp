#include "lio_builder.h"

namespace lio
{
    LioBuidler::LioBuidler()
    {

        init(params_);
    }
    LioBuidler::LioBuidler(LioParams &params) : params_(params)
    {
        init(params);
    }

    void LioBuidler::init(LioParams &params)
    {
        kf_.setMaxIter(params.max_iter);
        kf_.set_share_function([this](State &s, SharedState &d)
                               { computeResidual(s, d); });
        status_ = Status::INIT;
        first_lidar_ = true;
        Q_.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d::Identity() * params_.gyro_cov;
        Q_.block<3, 3>(3, 3).diagonal() = Eigen::Vector3d::Identity() * params_.acc_cov;
        Q_.block<3, 3>(6, 6).diagonal() = Eigen::Vector3d::Identity() * params_.gyro_bias_cov;
        Q_.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d::Identity() * params_.acc_bias_cov;

        last_acc_.setZero();
        last_gyro_.setZero();
    }

    void LioBuidler::processIMU(const IMUData &imu)
    {
        imus_.push_back(imu);
        if (status_ == Status::INIT)
        {
            if (imus_.size() < params_.init_imu_num)
                return;
            Eigen::Vector3d mean_acc(0, 0, 0);
            Eigen::Vector3d mean_gyro(0, 0, 0);
            for (auto &imu : imus_)
            {
                mean_acc += imu.acc;
                mean_gyro += imu.gyro;
            }
            mean_gyro = mean_gyro / static_cast<double>(imus_.size());
            mean_acc = mean_acc / static_cast<double>(imus_.size());
            init_gravity_norm_ = mean_acc.norm();
            kf_.x().bg = mean_gyro;
            kf_.x().pos_ext = params_.imu_ext_pos;
            kf_.x().rot_ext = params_.imu_ext_rot;
            kf_.x().initG(-mean_acc);

            if (params_.align_gravity)
            {
                kf_.x().rot = Eigen::Quaterniond::FromTwoVectors((-mean_acc).normalized(), Eigen::Vector3d(0.0, 0.0, -1.0)).matrix();
                kf_.x().initG(Eigen::Vector3d(0, 0, -1.0));
            }

            kf_.P().setIdentity();
            kf_.P()(6, 6) = kf_.P()(7, 7) = kf_.P()(8, 8) = 0.00001;
            kf_.P()(9, 9) = kf_.P()(10, 10) = kf_.P()(11, 11) = 0.00001;
            kf_.P()(15, 15) = kf_.P()(16, 16) = kf_.P()(17, 17) = 0.0001;
            kf_.P()(18, 18) = kf_.P()(19, 19) = kf_.P()(20, 20) = 0.001;
            kf_.P()(21, 21) = kf_.P()(22, 22) = 0.00001;
            last_imu_ = imus_.back();
            imus_.clear();
            status_ = Status::MAPPING;
        }
    }

    void LioBuidler::processLidar(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, double lidar_start_time)
    {
        // TODO: 有部分情况会跳过较多的IMU数据，开始需要静止
        if (status_ == Status::INIT)
            return;
        // 第一帧直接跳过
        if (first_lidar_)
        {
            first_lidar_ = false;
            last_imu_ = imus_.back();
            imus_.clear();
            last_lidar_end_time_ = lidar_start_time + cloud->points.back().curvature / 1000.0;
            return;
        }

        imus_.push_front(last_imu_);
        double imu_time_begin = imus_.front().sec;
        double imu_time_end = imus_.back().sec;
        double lidar_time_begin = lidar_start_time;
        double lidar_end_time = lidar_start_time + cloud->points.back().curvature / 1000.0;

        std::sort(cloud->points.begin(), cloud->points.end(), [](pcl::PointXYZINormal &p1, pcl::PointXYZINormal &p2) -> bool
                  { return p1.curvature < p2.curvature; });

        imu_poses_.clear();
        imu_poses_.emplace_back(0.0, last_acc_, last_gyro_, kf_.x().vel, kf_.x().pos, kf_.x().rot);

        Eigen::Vector3d acc_val, gyro_val;
        double dt = 0.0;
        Input inp;
        for (auto it_imu = imus_.begin(); it_imu < (imus_.end() - 1); it_imu++)
        {
            IMUData &head = *it_imu;
            IMUData &tail = *(it_imu + 1);
            if (tail.sec < last_lidar_end_time_)
                continue;
            gyro_val = 0.5 * (head.gyro + tail.gyro);
            acc_val = 0.5 * (head.acc + head.acc);
            acc_val = acc_val * 9.81 / init_gravity_norm_;

            if (head.sec < last_lidar_end_time_)
                dt = tail.sec - last_lidar_end_time_;
            else
                dt = tail.sec - head.sec;

            inp.acc = acc_val;
            inp.gyro = gyro_val;
            kf_.predict(inp, dt, Q_);
            last_gyro_ = gyro_val - kf_.x().bg;
            last_acc_ = kf_.x().rot * (acc_val - kf_.x().ba) + kf_.x().g;
            imu_poses_.emplace_back(tail.sec - lidar_time_begin, last_acc_, last_gyro_, kf_.x().vel, kf_.x().pos, kf_.x().rot);
        }

        dt = lidar_end_time - imu_time_end;
        kf_.predict(inp, dt, Q_);
        last_imu_ = imus_.back();
        imus_.clear();
        last_lidar_end_time_ = lidar_end_time;

        // 对齐到最后一帧上
        Eigen::Matrix3d cur_rot = kf_.x().rot;
        Eigen::Vector3d cur_pos = kf_.x().pos;
        Eigen::Matrix3d cur_rot_ext = kf_.x().rot_ext;
        Eigen::Vector3d cur_pos_ext = kf_.x().pos_ext;
        auto it_pcl = cloud->points.end() - 1;
        for (auto it_kp = imu_poses_.end() - 1; it_kp != imu_poses_.begin(); it_kp--)
        {
            auto head = it_kp - 1;
            auto tail = it_kp;
            Eigen::Matrix3d imu_rot = head->rot;
            Eigen::Vector3d imu_pos = head->pos;
            Eigen::Vector3d imu_vel = head->vel;
            Eigen::Vector3d imu_acc = tail->acc;
            Eigen::Vector3d imu_gyro = tail->gyro;
            for (; it_pcl->curvature / double(1000) > head->offset; it_pcl--)
            {
                dt = it_pcl->curvature / double(1000) - head->offset;
                Eigen::Vector3d point(it_pcl->x, it_pcl->y, it_pcl->z);
                Eigen::Matrix3d point_rot = imu_rot * Exp(imu_gyro * dt);
                Eigen::Vector3d point_pos = imu_pos + imu_vel * dt + 0.5 * imu_acc * dt * dt;
                Eigen::Vector3d p = cur_rot_ext.transpose() * (cur_rot.transpose() * (point_rot * (cur_rot_ext * point + cur_pos_ext) + point_pos - cur_pos) - cur_pos_ext);
                it_pcl->x = p(0);
                it_pcl->y = p(1);
                it_pcl->z = p(2);
                if (it_pcl == cloud->points.begin())
                    break;
            }
        }
    }

    void LioBuidler::computeResidual(State &state, SharedState &share_state)
    {
        
    }
    

} // namespace lio
