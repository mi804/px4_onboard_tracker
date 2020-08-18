#include <onboard_controller.h>
#include <math.h>
using namespace std;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::Twist current_velocity;
bool tracked;
int bbox[4];
int width;
int height;
struct coordinate
{
    double x;
    double y;
};

OnboardController::OnboardController(ros::NodeHandle nh_, ros::Rate rate_) : nh(nh_), rate(rate_)
{
    Initialize();
    Set_Offb_Mode();
    Arm();
    run();
}
OnboardController::~OnboardController()
{
    ROS_INFO("Destroying OnboardController......");
}

void OnboardController::run()
{

    // go to set_aptitude and then run tracking
    bool Test;
    ros::param::get("test", Test);
//    cout<<Test<<endl;
    if(Test)
    {
        test();
    }
    double set_aptitude = 4;
    while (ros::ok())
    {
        // ROS_INFO("current pose: (%f, %f, %f)",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
        Update_Target_Pose(0, 0, set_aptitude);
        if (fabs(current_pose.pose.position.z - set_aptitude) < 0.1)
        {
            ROS_INFO("get target aptitude");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    // run tracking
    while (ros::ok())
    {
        // ROS_INFO("current velocity: (%f, %f, %f)", current_velocity.linear.x, current_velocity.linear.y, current_velocity.linear.z);
        // Update_Target_Velocity(1, 1, 0);
        if (tracked)
        {
            coordinate tracking_center;
            coordinate image_center;
            coordinate velocity;
            tracking_center.x = 1.0 / 2 * (bbox[0] + bbox[2]);
            tracking_center.y = 1.0 / 2 * (bbox[1] + bbox[3]);
            image_center.x = width / 2;
            image_center.y = height / 2;
            double distance = sqrt((tracking_center.x - image_center.x) * (tracking_center.x - image_center.x) + (tracking_center.y - image_center.y) * (tracking_center.y - image_center.y));
            velocity.x = (tracking_center.x - image_center.x) / distance;
            velocity.y = (tracking_center.y - image_center.y) / distance;
            using namespace std;
            double dire_x, dire_y;
            ros::param::get("direction_x", dire_x);
            ros::param::get("direction_y", dire_y);
            velocity.x *= dire_x;
            velocity.y *= dire_y;
            ROS_INFO("Tracked, velocity:[%f, %f]", velocity.x, velocity.y);

            Update_Target_Velocity(velocity.x, velocity.y, 0);
        }
        else
        {
            ROS_INFO("no object!");
            Update_Target_Velocity(0, 0, 0);
        }

        ros::spinOnce();
        rate.sleep();
    }
}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
void velocity_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    current_velocity = *msg;
}

void tracker_cb(const uav_msgs::Tracking::ConstPtr &msg)
{
    // ROS_INFO("Whether catchs tracked: [%d]", msg->tracked);
    tracked = msg->tracked;
    for (int i = 0; i < 4; i++)
    {

        bbox[i] = msg->bbox[i];
    }
    width = msg->width;
    height = msg->height;
}

void OnboardController::Initialize()
{

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    velocity_sub = nh.subscribe<geometry_msgs::Twist>("/mavros/local_position/velocity", 10, velocity_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    tracking_msgs_sub = nh.subscribe<uav_msgs::Tracking>("commu_node/Tracking", 100, tracker_cb);

    // wait for FCU connection
    ROS_INFO("Connecting to FCU");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected!");

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        Update_Target_Pose(0, 0, 1, 0);
        ros::spinOnce();
        rate.sleep();
    }
}

void OnboardController::Set_Offb_Mode()
{
    mavros_msgs::SetMode offb_set_mode;
    ros::Time last_request = ros::Time::now();
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (current_state.mode != "OFFBOARD")
    {
        ROS_INFO("Calling offboard mode...");

        while (ros::ok())
        {
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled!");
                    break;
                }
                last_request = ros::Time::now();
            }
            Update_Target_Pose();
            ros::spinOnce();
            rate.sleep();
        }
    }
    else
    {
        ROS_INFO("Offboard Already enabled!");
    }
}
void OnboardController::Arm()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    if (current_state.armed)
    {
        ROS_INFO("Vehicle already armed!");
    }
    else
    {
        ROS_INFO("Arming vehicle....");
        while (ros::ok())
        {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                    break;
                }
                last_request = ros::Time::now();
            }
            Update_Target_Pose();
            ros::spinOnce();
            rate.sleep();
        }
    }
}
void OnboardController::Update_Target_Pose(double x, double y, double z, double yaw)
{
    double PI = 3.14159265358979323846264;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw / 180 * PI);
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    local_pos_pub.publish(pose);
}

void OnboardController::Update_Target_Velocity(double vx, double vy, double vz)
{
    velocity.linear.x = vx;
    velocity.linear.y = vy;
    velocity.linear.z = vz;
    velocity_pub.publish(velocity);
}

void OnboardController::test()
{

    // go to set_aptitude and then run tracking
    double set_aptitude = 4;
    double set_x1 = 1.0;
    double set_x2 = -1.0;
    double set_y1 = 1.0;
    double set_y2 = -1.0;

    while (ros::ok())
    {
        // ROS_INFO("current pose: (%f, %f, %f)",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
        Update_Target_Pose(0, 0, set_aptitude);
        if (fabs(current_pose.pose.position.z - set_aptitude) < 0.1)
        {
            ROS_INFO("get target aptitude");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    //go to the set position(1,0,4)
    while (ros::ok())
    {
        // ROS_INFO("current pose: (%f, %f, %f)",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
        Update_Target_Pose(set_x1, 0, set_aptitude);
        if (fabs(current_pose.pose.position.x - set_x1) < 0.1)
        {
            ROS_INFO("get target position (1,0,4)");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {
        // ROS_INFO("current pose: (%f, %f, %f)",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
        Update_Target_Pose(set_x2, 0, set_aptitude);
        if (fabs(current_pose.pose.position.x - set_x2) < 0.1)
        {
            ROS_INFO("get target position (-1,0,4)");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {
        // ROS_INFO("current pose: (%f, %f, %f)",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
        Update_Target_Pose(set_x2, set_y1, set_aptitude);
        if (fabs(current_pose.pose.position.y - set_y1) < 0.1)
        {
            ROS_INFO("get target position (-1,1,4)");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {
        // ROS_INFO("current pose: (%f, %f, %f)",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
        Update_Target_Pose(set_x2, set_y2, set_aptitude);
        if (fabs(current_pose.pose.position.y - set_y2) < 0.1)
        {
            ROS_INFO("get target position (-1,-1,4)");
            // break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    // while (ros::ok())
    // {
    //     // ROS_INFO("current pose: (%f, %f, %f)",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
    //     Update_Target_Pose(0, 0, 0);
    //     double distance = current_pose.pose.position.x * current_pose.pose.position.x + current_pose.pose.position.y * current_pose.pose.position.y + current_pose.pose.position.z * current_pose.pose.position.z;
    //     if (fabs(distance) < 0.1)
    //     {
    //         ROS_INFO("get target position (0,0,0)");
    //         break;
    //     }
    //     ros::spinOnce();
    //     rate.sleep();
    // }
}