#include "commons.h"
namespace lio
{

    Eigen::Vector3d rotate2rpy(Eigen::Matrix3d &rot)
    {
        double roll = std::atan2(rot(2, 1), rot(2, 2));
        double pitch = asin(-rot(2, 0));
        double yaw = std::atan2(rot(1, 0), rot(0, 0));
        return Eigen::Vector3d(roll, pitch, yaw);
    }

    float sq_dist(const pcl::PointXYZINormal &p1, const pcl::PointXYZINormal &p2)
    {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    }

}
