#include <ros/ros.h>
#include <onboard_controller.h>

using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "onboard_controller_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    OnboardController Controller(nh, rate);
    ros::spin();
    return 0;
}