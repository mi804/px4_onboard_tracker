#ifndef ONBOARD_CONTROLLER_H
#define ONBOARD_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "uav_msgs/Tracking.h"
#include <iostream>
#include <tf/tf.h>
using namespace std;

extern mavros_msgs::State current_state;
extern geometry_msgs::PoseStamped current_pose;
extern geometry_msgs::Twist current_velocity;
void state_cb(const mavros_msgs::State::ConstPtr &msg);

class OnboardController
{
public:
    ros::Subscriber state_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber tracking_msgs_sub;
    ros::Publisher local_pos_pub;
    ros::Publisher velocity_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist velocity;
    ros::NodeHandle nh;
    ros::Rate rate;
    void Initialize();
    void Set_Offb_Mode();
    void Arm();
    void Update_Target_Pose(double x = 0, double y = 0, double z = 2, double yaw = 0);
    void Update_Target_Velocity(double vx = 0, double vy = 0, double vz = 0);
    void run();
    OnboardController(ros::NodeHandle nh, ros::Rate rate_);
    ~OnboardController();
};

#endif
