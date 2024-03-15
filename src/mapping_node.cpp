#include <ros/ros.h>

struct RosParams
{
    std::string lidar_topic;
    std::string imu_topic;
    double lio_hz = 50;
};

class LioMappingNode
{

public:
    LioMappingNode(ros::NodeHandle &nh) : nh_(nh)
    {

    }
    void initSubScribers()
    {

    }
    void init_params()
    {
        
    }

private:
    ros::NodeHandle &nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_node");
}