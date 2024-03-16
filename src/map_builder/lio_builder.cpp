#include "lio_builder.h"

namespace lio
{
    float sq_dist(const pcl::PointXYZINormal &p1, const pcl::PointXYZINormal &p2)
    {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    }

    bool esti_plane(Eigen::Vector4d &out, const KD_TREE<pcl::PointXYZINormal>::PointVector &points, const double &thresh)
    {
        Eigen::Matrix<double, 5, 3> A;
        Eigen::Matrix<double, 5, 1> b;
        A.setZero();
        b.setOnes();
        b *= -1.0;
        for (int i = 0; i < 5; i++)
        {
            A(i, 0) = points[i].x;
            A(i, 1) = points[i].y;
            A(i, 2) = points[i].z;
        }

        Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);

        double norm = normvec.norm();
        out[0] = normvec(0) / norm;
        out[1] = normvec(1) / norm;
        out[2] = normvec(2) / norm;
        out[3] = 1.0 / norm;

        for (int j = 0; j < 5; j++)
        {
            if (std::fabs(out(0) * points[j].x + out(1) * points[j].y + out(2) * points[j].z + out(3)) > thresh)
            {
                return false;
            }
        }
        return true;
    }

    LioBuilder::LioBuilder()
    {
        init(params_);
    }

    LioBuilder::LioBuilder(LioParams &params) : params_(params)
    {
        init(params);
    }

    void LioBuilder::init(LioParams &params)
    {
        kf_.setMaxIter(params.max_iter);
        kf_.set_share_function([this](State &s, SharedState &d)
                               { computeResidual(s, d); });
        status_ = Status::INIT;
        first_lidar_ = true;
        map_init_ = false;
        Q_.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d::Identity() * params_.gyro_cov;
        Q_.block<3, 3>(3, 3).diagonal() = Eigen::Vector3d::Identity() * params_.acc_cov;
        Q_.block<3, 3>(6, 6).diagonal() = Eigen::Vector3d::Identity() * params_.gyro_bias_cov;
        Q_.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d::Identity() * params_.acc_bias_cov;

        last_acc_.setZero();
        last_gyro_.setZero();

        ikdtree_ = std::make_shared<KD_TREE<pcl::PointXYZINormal>>();
        ikdtree_->set_downsample_param(params.map_resolution);
        scan_filter_.setLeafSize(params.scan_resolution, params.scan_resolution, params.scan_resolution);

        cloud_down_lidar_.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
        cloud_down_world_.reset(new pcl::PointCloud<pcl::PointXYZINormal>(10000, 1));
        norm_vec_.reset(new pcl::PointCloud<pcl::PointXYZINormal>(10000, 1));
        effect_cloud_lidar_.reset(new pcl::PointCloud<pcl::PointXYZINormal>(10000, 1));
        effect_norm_vec_.reset(new pcl::PointCloud<pcl::PointXYZINormal>(10000, 1));
        nearest_points_.resize(10000);
        point_selected_flag_.resize(10000, false);
    }

    void LioBuilder::processIMU(const IMUData &imu)
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

    void LioBuilder::undistortCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, double lidar_start_time)
    {
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

    void LioBuilder::processLidar(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, double lidar_start_time)
    {
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

        undistortCloud(cloud, lidar_start_time);

        scan_filter_.setInputCloud(cloud);
        scan_filter_.filter(*cloud_down_lidar_);
        if (!map_init_)
        {
            std::cout << "build map" << std::endl;
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_world = transformToWorld(cloud_down_lidar_);
            ikdtree_->Build(point_world->points);
            std::cout << "map points: " << point_world->points.size() << std::endl;
            map_init_ = true;
            return;
        }
        trimMap();
        // std::cout << "before update " << std::endl;
        kf_.update();
        // std::cout << "after update " << std::endl;
        increaseMap();
    }

    void LioBuilder::increaseMap()
    {
        if (status_ == Status::INIT)
            return;
        if (cloud_down_lidar_->empty())
            return;
        int size = cloud_down_lidar_->size();
        KD_TREE<pcl::PointXYZINormal>::PointVector point_to_add;
        KD_TREE<pcl::PointXYZINormal>::PointVector point_no_need_downsample;
        point_to_add.reserve(size);
        point_no_need_downsample.reserve(size);

        for (int i = 0; i < size; i++)
        {
            const pcl::PointXYZINormal &p = cloud_down_lidar_->points[i];
            Eigen::Vector3d point(p.x, p.y, p.z);
            point = kf_.x().rot * (kf_.x().rot_ext * point + kf_.x().pos_ext) + kf_.x().pos;
            cloud_down_world_->points[i].x = point(0);
            cloud_down_world_->points[i].y = point(1);
            cloud_down_world_->points[i].z = point(2);
            cloud_down_world_->points[i].intensity = cloud_down_lidar_->points[i].intensity;
            // 如果该点附近没有近邻点则需要添加到地图中
            if (nearest_points_[i].empty())
            {
                point_to_add.push_back(cloud_down_world_->points[i]);
                continue;
            }

            const KD_TREE<pcl::PointXYZINormal>::PointVector &points_near = nearest_points_[i];
            bool need_add = true;
            pcl::PointXYZINormal downsample_result, mid_point;
            mid_point.x = std::floor(cloud_down_world_->points[i].x / params_.map_resolution) * params_.map_resolution + 0.5 * params_.map_resolution;
            mid_point.y = std::floor(cloud_down_world_->points[i].y / params_.map_resolution) * params_.map_resolution + 0.5 * params_.map_resolution;
            mid_point.z = std::floor(cloud_down_world_->points[i].z / params_.map_resolution) * params_.map_resolution + 0.5 * params_.map_resolution;

            // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * params_.map_resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * params_.map_resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * params_.map_resolution)
            {
                point_no_need_downsample.push_back(cloud_down_world_->points[i]);
                continue;
            }
            float dist = sq_dist(cloud_down_world_->points[i], mid_point);
            for (int readd_i = 0; readd_i < 5; readd_i++)
            {
                // 如果该点的近邻点较少，则需要加入到地图中
                if (points_near.size() < 5)
                    break;
                // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
                if (sq_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                point_to_add.push_back(cloud_down_world_->points[i]);
        }
        int add_point_size = ikdtree_->Add_Points(point_to_add, true);
        ikdtree_->Add_Points(point_no_need_downsample, false);
    }

    void LioBuilder::trimMap()
    {
        local_map_.cub_to_rm.clear();
        Eigen::Vector3d pos_lidar = kf_.x().pos + kf_.x().rot * kf_.x().pos_ext;
        if (!local_map_.is_initialed)
        {
            for (int i = 0; i < 3; i++)
            {
                local_map_.local_map_corner.vertex_min[i] = pos_lidar[i] - local_map_.cube_len / 2.0;
                local_map_.local_map_corner.vertex_max[i] = pos_lidar[i] + local_map_.cube_len / 2.0;
            }
            local_map_.is_initialed = true;
            return;
        }
        float dist_to_map_edge[3][2];
        bool need_move = false;
        double det_thresh = local_map_.move_thresh * local_map_.det_range;
        for (int i = 0; i < 3; i++)
        {
            dist_to_map_edge[i][0] = fabs(pos_lidar(i) - local_map_.local_map_corner.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_lidar(i) - local_map_.local_map_corner.vertex_max[i]);

            if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
                need_move = true;
        }
        if (!need_move)
            return;
        BoxPointType new_corner, temp_corner;
        new_corner = local_map_.local_map_corner;
        float mov_dist = std::max((local_map_.cube_len - 2.0 * local_map_.move_thresh * local_map_.det_range) * 0.5 * 0.9,
                                  double(local_map_.det_range * (local_map_.move_thresh - 1)));
        for (int i = 0; i < 3; i++)
        {
            temp_corner = local_map_.local_map_corner;
            if (dist_to_map_edge[i][0] <= det_thresh)
            {
                new_corner.vertex_max[i] -= mov_dist;
                new_corner.vertex_min[i] -= mov_dist;
                temp_corner.vertex_min[i] = local_map_.local_map_corner.vertex_max[i] - mov_dist;
                local_map_.cub_to_rm.push_back(temp_corner);
            }
            else if (dist_to_map_edge[i][1] <= det_thresh)
            {
                new_corner.vertex_max[i] += mov_dist;
                new_corner.vertex_min[i] += mov_dist;
                temp_corner.vertex_max[i] = local_map_.local_map_corner.vertex_min[i] + mov_dist;
                local_map_.cub_to_rm.push_back(temp_corner);
            }
        }
        local_map_.local_map_corner = new_corner;
        KD_TREE<pcl::PointXYZINormal>::PointVector points_history;
        ikdtree_->acquire_removed_points(points_history);
        if (local_map_.cub_to_rm.size() > 0)
            ikdtree_->Delete_Point_Boxes(local_map_.cub_to_rm);
        return;
    }

    void LioBuilder::computeResidual(State &state, SharedState &share_state)
    {
        int size = cloud_down_lidar_->size();
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for

#endif
        for (int i = 0; i < size; i++)
        {
            pcl::PointXYZINormal &point_body = cloud_down_lidar_->points[i];
            pcl::PointXYZINormal &point_world = cloud_down_world_->points[i];
            Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);

            Eigen::Vector3d point_world_vec = state.rot * (state.rot_ext * point_body_vec + state.pos_ext) + state.pos;

            point_world.x = point_body_vec.x();
            point_world.y = point_world_vec(1);
            point_world.z = point_world_vec(2);
            point_world.intensity = point_body.intensity;

            std::vector<float> point_sq_dist(5);

            auto &points_near = nearest_points_[i];

            ikdtree_->Nearest_Search(point_world, 5, points_near, point_sq_dist);

            if (points_near.size() >= 5 && point_sq_dist[4] <= 5)
                point_selected_flag_[i] = true;
            else
                point_selected_flag_[i] = false;

            if (!point_selected_flag_[i])
                continue;
            Eigen::Vector4d pabcd;
            point_selected_flag_[i] = false;

            if (esti_plane(pabcd, points_near, 0.1))
            {
                double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) + pabcd(3);
                // 和点面距离正相关，和点的远近距离负相关
                double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
                if (s > 0.9)
                {
                    point_selected_flag_[i] = true;
                    norm_vec_->points[i].x = pabcd(0);
                    norm_vec_->points[i].y = pabcd(1);
                    norm_vec_->points[i].z = pabcd(2);
                    norm_vec_->points[i].intensity = pd2;
                }
            }
        }

        int effect_feat_num = 0;
        for (int i = 0; i < size; i++)
        {
            if (!point_selected_flag_[i])
                continue;
            effect_cloud_lidar_->points[effect_feat_num] = cloud_down_lidar_->points[i];
            effect_norm_vec_->points[effect_feat_num] = norm_vec_->points[i];
            effect_feat_num++;
        }

        share_state.H.setZero();
        share_state.b.setZero();
        if (effect_feat_num < 1)
        {
            std::cerr << "no effective points" << std::endl;
            return;
        }

        Eigen::Matrix<double, 1, 12> J;
        for (int i = 0; i < effect_feat_num; i++)
        {
            J.setZero();
            const pcl::PointXYZINormal &laser_p = effect_cloud_lidar_->points[i];
            const pcl::PointXYZINormal &norm_p = effect_norm_vec_->points[i];
            Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
            Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
            Eigen::Matrix<double, 1, 3> B = -norm_vec.transpose() * state.rot * skew(state.rot_ext * laser_p_vec + state.pos_ext);
            J.block<1, 3>(0, 0) = norm_vec.transpose();
            J.block<1, 3>(0, 3) = B;

            if (1)
            {
                Eigen::Matrix<double, 1, 3> C = -norm_vec.transpose() * state.rot * state.rot_ext * skew(laser_p_vec);
                Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * state.rot;
                J.block<1, 3>(0, 6) = C;
                J.block<1, 3>(0, 9) = D;
            }
            share_state.H += J.transpose() * 1000 * J;
            share_state.b += J.transpose() * 1000 * norm_p.intensity;
        }
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr LioBuilder::transformToWorld(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZINormal>);
        Eigen::Matrix3d rot = kf_.x().rot;
        Eigen::Vector3d pos = kf_.x().pos;
        Eigen::Matrix3d rot_ext = kf_.x().rot_ext;
        Eigen::Vector3d pos_ext = kf_.x().pos_ext;
        cloud_world->reserve(cloud->size());
        for (auto &p : cloud->points)
        {
            Eigen::Vector3d point(p.x, p.y, p.z);
            point = rot * (rot_ext * point + pos_ext) + pos;
            pcl::PointXYZINormal p_world;
            p_world.x = point(0);
            p_world.y = point(1);
            p_world.z = point(2);
            p_world.intensity = p.intensity;
            cloud_world->points.push_back(p_world);
        }
        return cloud_world;
    }
} // namespace lio
