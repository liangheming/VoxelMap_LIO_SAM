#include "lio_builder.h"

namespace lio
{

    void LioBuilder::initialize(LIOParams &params)
    {
        params_ = params;
        status_ = LIOStatus::IMU_INIT;
        data_group_.Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * params.ng;
        data_group_.Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * params.na;
        data_group_.Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * params.nbg;
        data_group_.Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * params.nba;
        fastlio_data_.initialize(params.map_resolution);

        kf_.set_share_function(
            [&](kf::State &s, kf::SharedState &d)
            { fastlio_data_.sharedUpdateFunc(s, d); });

        scan_filter_.setLeafSize(params.scan_resolution, params.scan_resolution, params.scan_resolution);
    }

    void LioBuilder::operator()(SyncPackage &package)
    {

        if (status_ == LIOStatus::IMU_INIT)
        {
            if (initializeImu(package.imus))
            {
                status_ = LIOStatus::MAP_INIT;
                data_group_.last_cloud_end_time = package.cloud_end_time;
            }
        }
        else if (status_ == LIOStatus::MAP_INIT)
        {
            undistortCloud(package);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_world = transformToWorld(package.cloud);
            fastlio_data_.ikdtree->Build(point_world->points);
            status_ = LIOStatus::LIO_MAPPING;
        }
        else
        {
            scan_filter_.setInputCloud(package.cloud);
            scan_filter_.filter(*package.cloud);
            undistortCloud(package);
            fastlio_data_.cloud_down_lidar = package.cloud;
            fastlio_data_.trimMap(kf_.x());
            kf_.update();
            fastlio_data_.increaseMap(kf_.x());
        }
    }

    bool LioBuilder::initializeImu(std::vector<IMUData> &imus)
    {
        data_group_.imu_cache.insert(data_group_.imu_cache.end(), imus.begin(), imus.end());
        if (data_group_.imu_cache.size() < params_.imu_init_num)
            return false;
        Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
        for (const auto &imu : data_group_.imu_cache)
        {
            acc_mean += imu.acc;
            gyro_mean += imu.gyro;
        }
        acc_mean /= static_cast<double>(data_group_.imu_cache.size());
        gyro_mean /= static_cast<double>(data_group_.imu_cache.size());
        data_group_.gravity_norm = acc_mean.norm();
        kf_.x().rot_ext = params_.r_il;
        kf_.x().pos_ext = params_.p_il;
        kf_.x().bg = gyro_mean;
        if (params_.gravity_align)
        {
            kf_.x().rot = (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), Eigen::Vector3d(0.0, 0.0, -1.0)).matrix());
            kf_.x().initG(Eigen::Vector3d(0, 0, -1.0));
        }
        else
        {
            kf_.x().initG(-acc_mean);
        }
        kf_.P().setIdentity();
        kf_.P().block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.00001;
        kf_.P().block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.00001;
        kf_.P().block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() * 0.0001;
        kf_.P().block<3, 3>(18, 18) = Eigen::Matrix3d::Identity() * 0.0001;
        kf_.P().block<2, 2>(21, 21) = Eigen::Matrix2d::Identity() * 0.00001;
        data_group_.last_imu = imus.back();
        return true;
    }

    void LioBuilder::undistortCloud(SyncPackage &package)
    {
        data_group_.imu_cache.clear();
        data_group_.imu_cache.push_back(data_group_.last_imu);
        data_group_.imu_cache.insert(data_group_.imu_cache.end(), package.imus.begin(), package.imus.end());

        const double imu_time_begin = data_group_.imu_cache.front().timestamp;
        const double imu_time_end = data_group_.imu_cache.back().timestamp;
        const double cloud_time_begin = package.cloud_start_time;
        const double cloud_time_end = package.cloud_end_time;

        std::sort(package.cloud->points.begin(), package.cloud->points.end(), [](pcl::PointXYZINormal &p1, pcl::PointXYZINormal &p2) -> bool
                  { return p1.curvature < p2.curvature; });

        data_group_.imu_poses_cache.clear();
        data_group_.imu_poses_cache.emplace_back(0.0, data_group_.last_acc, data_group_.last_gyro,
                                                 kf_.x().vel, kf_.x().pos, kf_.x().rot);

        Eigen::Vector3d acc_val, gyro_val;
        double dt = 0.0;
        kf::Input inp;

        for (auto it_imu = data_group_.imu_cache.begin(); it_imu < (data_group_.imu_cache.end() - 1); it_imu++)
        {
            IMUData &head = *it_imu;
            IMUData &tail = *(it_imu + 1);

            if (tail.timestamp < data_group_.last_cloud_end_time)
                continue;
            gyro_val = 0.5 * (head.gyro + tail.gyro);
            acc_val = 0.5 * (head.acc + tail.acc);

            acc_val = acc_val * 9.81 / data_group_.gravity_norm;

            if (head.timestamp < data_group_.last_cloud_end_time)
                dt = tail.timestamp - data_group_.last_cloud_end_time;
            else
                dt = tail.timestamp - head.timestamp;

            inp.acc = acc_val;
            inp.gyro = gyro_val;

            kf_.predict(inp, dt, data_group_.Q);

            data_group_.last_gyro = gyro_val - kf_.x().bg;
            data_group_.last_acc = kf_.x().rot * (acc_val - kf_.x().ba) + kf_.x().g;

            double offset = tail.timestamp - cloud_time_begin;
            data_group_.imu_poses_cache.emplace_back(offset, data_group_.last_acc, data_group_.last_gyro,
                                                     kf_.x().vel, kf_.x().pos, kf_.x().rot);
        }

        dt = cloud_time_end - imu_time_end;

        kf_.predict(inp, dt, data_group_.Q);

        data_group_.last_imu = package.imus.back();
        data_group_.last_cloud_end_time = cloud_time_end;

        Eigen::Matrix3d cur_rot = kf_.x().rot;
        Eigen::Vector3d cur_pos = kf_.x().pos;
        Eigen::Matrix3d cur_rot_ext = kf_.x().rot_ext;
        Eigen::Vector3d cur_pos_ext = kf_.x().pos_ext;

        auto it_pcl = package.cloud->points.end() - 1;
        for (auto it_kp = data_group_.imu_poses_cache.end() - 1; it_kp != data_group_.imu_poses_cache.begin(); it_kp--)
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
                Eigen::Matrix3d point_rot = imu_rot * Sophus::SO3d::exp(imu_gyro * dt).matrix();
                Eigen::Vector3d point_pos = imu_pos + imu_vel * dt + 0.5 * imu_acc * dt * dt;
                Eigen::Vector3d p_compensate = cur_rot_ext.transpose() * (cur_rot.transpose() * (point_rot * (cur_rot_ext * point + cur_pos_ext) + point_pos - cur_pos) - cur_pos_ext);
                it_pcl->x = p_compensate(0);
                it_pcl->y = p_compensate(1);
                it_pcl->z = p_compensate(2);

                if (it_pcl == package.cloud->points.begin())
                    break;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr LioBuilder::transformToWorld(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZINormal>);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = (kf_.x().rot * kf_.x().rot_ext).cast<float>();
        transform.block<3, 1>(0, 3) = (kf_.x().rot * kf_.x().pos_ext + kf_.x().pos).cast<float>();
        pcl::transformPointCloud(*cloud, *cloud_world, transform);
        return cloud_world;
    }

    //===================================================
    void FASTLIODataGroup::initialize(double _resolution)
    {
        ikdtree = std::make_shared<KD_TREE<pcl::PointXYZINormal>>();
        ikdtree->set_downsample_param(_resolution);
        resolution = _resolution;
        cloud_down_lidar.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
        cloud_down_world.reset(new pcl::PointCloud<pcl::PointXYZINormal>(10000, 1));
        norm_vec.reset(new pcl::PointCloud<pcl::PointXYZINormal>(10000, 1));
        effect_cloud_lidar.reset(new pcl::PointCloud<pcl::PointXYZINormal>(10000, 1));
        effect_norm_vec.reset(new pcl::PointCloud<pcl::PointXYZINormal>(10000, 1));
        nearest_points.resize(10000);
        point_selected_flag.resize(10000, false);
    }

    bool FASTLIODataGroup::esti_plane(Eigen::Vector4d &out, const KD_TREE<pcl::PointXYZINormal>::PointVector &points, const double &thresh)
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

    void FASTLIODataGroup::trimMap(kf::State state)
    {
        local_map.cub_to_rm.clear();
        Eigen::Vector3d pos_lidar = state.pos + state.rot * state.pos_ext;
        // 根据lidar的位置进行局部地图的初始化
        if (!local_map.is_initialed)
        {
            for (int i = 0; i < 3; i++)
            {
                local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - local_map.cube_len / 2.0;
                local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + local_map.cube_len / 2.0;
            }
            local_map.is_initialed = true;
            return;
        }

        float dist_to_map_edge[3][2];
        bool need_move = false;
        double det_thresh = local_map.move_thresh * local_map.det_range;
        // 如果靠近地图边缘 则需要进行地图的移动
        for (int i = 0; i < 3; i++)
        {
            dist_to_map_edge[i][0] = fabs(pos_lidar(i) - local_map.local_map_corner.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_lidar(i) - local_map.local_map_corner.vertex_max[i]);

            if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
                need_move = true;
        }
        if (!need_move)
            return;

        BoxPointType new_corner, temp_corner;
        new_corner = local_map.local_map_corner;
        float mov_dist = std::max((local_map.cube_len - 2.0 * local_map.move_thresh * local_map.det_range) * 0.5 * 0.9,
                                  double(local_map.det_range * (local_map.move_thresh - 1)));
        // 更新局部地图
        for (int i = 0; i < 3; i++)
        {
            temp_corner = local_map.local_map_corner;
            if (dist_to_map_edge[i][0] <= det_thresh)
            {
                new_corner.vertex_max[i] -= mov_dist;
                new_corner.vertex_min[i] -= mov_dist;
                temp_corner.vertex_min[i] = local_map.local_map_corner.vertex_max[i] - mov_dist;
                local_map.cub_to_rm.push_back(temp_corner);
            }
            else if (dist_to_map_edge[i][1] <= det_thresh)
            {
                new_corner.vertex_max[i] += mov_dist;
                new_corner.vertex_min[i] += mov_dist;
                temp_corner.vertex_max[i] = local_map.local_map_corner.vertex_min[i] + mov_dist;
                local_map.cub_to_rm.push_back(temp_corner);
            }
        }
        local_map.local_map_corner = new_corner;
        // 强制删除历史点云
        KD_TREE<pcl::PointXYZINormal>::PointVector points_history;
        ikdtree->acquire_removed_points(points_history);

        // 删除局部地图之外的点云
        if (local_map.cub_to_rm.size() > 0)
            ikdtree->Delete_Point_Boxes(local_map.cub_to_rm);
        return;
    }

    void FASTLIODataGroup::increaseMap(kf::State state)
    {
        if (cloud_down_lidar->empty())
            return;
        int size = cloud_down_lidar->size();

        KD_TREE<pcl::PointXYZINormal>::PointVector point_to_add;
        KD_TREE<pcl::PointXYZINormal>::PointVector point_no_need_downsample;

        point_to_add.reserve(size);
        point_no_need_downsample.reserve(size);

        for (int i = 0; i < size; i++)
        {
            const pcl::PointXYZINormal &p = cloud_down_lidar->points[i];
            Eigen::Vector3d point(p.x, p.y, p.z);
            point = state.rot * (state.rot_ext * point + state.pos_ext) + state.pos;
            cloud_down_world->points[i].x = point(0);
            cloud_down_world->points[i].y = point(1);
            cloud_down_world->points[i].z = point(2);
            cloud_down_world->points[i].intensity = cloud_down_lidar->points[i].intensity;
            // 如果该点附近没有近邻点则需要添加到地图中
            if (nearest_points[i].empty())
            {
                point_to_add.push_back(cloud_down_world->points[i]);
                continue;
            }

            const KD_TREE<pcl::PointXYZINormal>::PointVector &points_near = nearest_points[i];
            bool need_add = true;
            pcl::PointXYZINormal downsample_result, mid_point;
            mid_point.x = std::floor(cloud_down_world->points[i].x / resolution) * resolution + 0.5 * resolution;
            mid_point.y = std::floor(cloud_down_world->points[i].y / resolution) * resolution + 0.5 * resolution;
            mid_point.z = std::floor(cloud_down_world->points[i].z / resolution) * resolution + 0.5 * resolution;

            // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * resolution)
            {
                point_no_need_downsample.push_back(cloud_down_world->points[i]);
                continue;
            }
            float dist = sq_dist(cloud_down_world->points[i], mid_point);

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
                point_to_add.push_back(cloud_down_world->points[i]);
        }
        int add_point_size = ikdtree->Add_Points(point_to_add, true);
        ikdtree->Add_Points(point_no_need_downsample, false);
    }

    void FASTLIODataGroup::sharedUpdateFunc(kf::State &state, kf::SharedState &share_data)
    {
        int size = cloud_down_lidar->size();
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for

#endif
        for (int i = 0; i < size; i++)
        {
            pcl::PointXYZINormal &point_body = cloud_down_lidar->points[i];
            pcl::PointXYZINormal &point_world = cloud_down_world->points[i];
            Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
            Eigen::Vector3d point_world_vec = state.rot * (state.rot_ext * point_body_vec + state.pos_ext) + state.pos;
            point_world.x = point_world_vec(0);
            point_world.y = point_world_vec(1);
            point_world.z = point_world_vec(2);
            point_world.intensity = point_body.intensity;

            std::vector<float> point_sq_dist(5);
            auto &points_near = nearest_points[i];

            ikdtree->Nearest_Search(point_world, 5, points_near, point_sq_dist);
            if (points_near.size() >= 5 && point_sq_dist[5 - 1] <= 5)
                point_selected_flag[i] = true;
            else
                point_selected_flag[i] = false;

            if (!point_selected_flag[i])
                continue;

            Eigen::Vector4d pabcd;
            point_selected_flag[i] = false;

            // 估计平面法向量，同时计算点面距离，计算的值存入intensity
            if (esti_plane(pabcd, points_near, 0.1))
            {
                double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) + pabcd(3);
                // 和点面距离正相关，和点的远近距离负相关
                double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
                if (s > 0.9)
                {
                    point_selected_flag[i] = true;
                    norm_vec->points[i].x = pabcd(0);
                    norm_vec->points[i].y = pabcd(1);
                    norm_vec->points[i].z = pabcd(2);
                    norm_vec->points[i].intensity = pd2;
                }
            }
        }

        int effect_feat_num = 0;
        for (int i = 0; i < size; i++)
        {
            if (!point_selected_flag[i])
                continue;
            effect_cloud_lidar->points[effect_feat_num] = cloud_down_lidar->points[i];
            effect_norm_vec->points[effect_feat_num] = norm_vec->points[i];
            effect_feat_num++;
        }

        share_data.H.setZero();
        share_data.b.setZero();
        if (effect_feat_num < 1)
        {
            std::cout << "NO Effective Points!" << std::endl;
            return;
        }
        Eigen::Matrix<double, 1, 12> J;
        for (int i = 0; i < effect_feat_num; i++)
        {
            J.setZero();
            const pcl::PointXYZINormal &laser_p = effect_cloud_lidar->points[i];
            const pcl::PointXYZINormal &norm_p = effect_norm_vec->points[i];
            Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
            Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
            Eigen::Matrix<double, 1, 3> B = -norm_vec.transpose() * state.rot * Sophus::SO3d::hat(state.rot_ext * laser_p_vec + state.pos_ext);
            J.block<1, 3>(0, 0) = norm_vec.transpose();
            J.block<1, 3>(0, 3) = B;

            if (1)
            {
                Eigen::Matrix<double, 1, 3> C = -norm_vec.transpose() * state.rot * state.rot_ext * Sophus::SO3d::hat(laser_p_vec);
                Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * state.rot;
                J.block<1, 3>(0, 6) = C;
                J.block<1, 3>(0, 9) = D;
            }
            share_data.H += J.transpose() * 1000 * J;
            share_data.b += J.transpose() * 1000 * norm_p.intensity;
        }
    }
} // namespace lio
