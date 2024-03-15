#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("HELLOW ROS!");
    }
    return 0;
}