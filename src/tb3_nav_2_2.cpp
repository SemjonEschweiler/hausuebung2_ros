// After various trial and errors I came to the conclusion to use following control parameters:
// k_linear = 0.25; <-- this is faster than usual (usual for HÜ2.1 is 0.125) but for the application it works because no sharp turns will needed to be performed and it get's the robot faster to the endgoal
// k_linear is also > 0
// k_alpha = 0.8; <-- abs(k_alpha) needs to be bigger than abs(k_linear) see Siegwart: Introduction to mobile robots
// k_beta = -0.5; <-- beta is smaller than 0 see Siegwart: Introduction to mobile robots
//

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"
#include <sstream>


#include "dynamic_reconfigure/server.h"
#include "hausuebung2_semjon_eschweiler/dynamicParamsConfig.h"


struct Quaternion{
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

struct PoseTurtlebot3 {
    double x, y, th;
};

ros::Publisher publisher;
// ros::Publisher pose_publisher;
ros::Subscriber subscriber_pose;
ros::ServiceClient client;
nav_msgs::Odometry turtle_odom;
std_msgs::String pose_info;
geometry_msgs::Twist vel_msg;
PoseTurtlebot3 current_pose;
double PI = 3.1415926535897;
ros::Time current_time, last_time;
bool checkbox_start_status_earlier = false;
bool checkbox_reset_status_earlier = false;
bool reset_is_set = false;
double k_linear, k_alpha, k_beta;

void moveStraightLine(double distance, bool moveForward, double velocity);
void moveStraightLineOdom(double distance, bool moveForward, double velocity);
void rotateByAngle(double angleDg, bool positiveRot, double angVelocityDg);
void pose_callback(const nav_msgs::Odometry::ConstPtr & pose_message);
double put_angle_in_range(double angle);
void go_to_goal(PoseTurtlebot3 goal);
void endMovement();
double rad2deg(double rad);
double deg2rad(double deg);
EulerAngles ToEulerAngles(Quaternion q);

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "tb3_nav_2_2_node");
	ros::NodeHandle n;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

	publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    subscriber_pose = n.subscribe("/odom", 10, pose_callback); //Why does pose_callback not get called sometimes?
    client = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");

    //Test your code here

    k_linear = 0.25;
    k_alpha = 0.8;
    k_beta = -0.5;
    PoseTurtlebot3 xi_1;
    PoseTurtlebot3 xi_2;
    PoseTurtlebot3 xi_3;
    PoseTurtlebot3 xi_4;

    n.getParam("x_1", xi_1.x);
    n.getParam("y_1", xi_1.y);
    n.getParam("th_dg_1", xi_1.th);

    n.getParam("x_2", xi_2.x);
    n.getParam("y_2", xi_2.y);
    n.getParam("th_dg_2", xi_2.th);

    n.getParam("x_3", xi_3.x);
    n.getParam("y_3", xi_3.y);
    n.getParam("th_dg_3", xi_3.th);

    n.getParam("x_4", xi_4.x);
    n.getParam("y_4", xi_4.y);
    n.getParam("th_dg_4", xi_4.th);

    xi_1.th = deg2rad(xi_1.th);
    xi_2.th = deg2rad(xi_2.th);
    xi_3.th = deg2rad(xi_3.th);
    xi_4.th = deg2rad(xi_4.th);


	ROS_INFO("\n\n\n******START TESTING************\n");
	ros::Rate loop_rate(10);
    go_to_goal(xi_1);
    go_to_goal(xi_2);
    go_to_goal(xi_3);
    go_to_goal(xi_4);
    go_to_goal(xi_1);
    ros::shutdown();
    ros::spin();

   return 0;
}


void pose_callback(const nav_msgs::Odometry::ConstPtr & pose_message){
    // ros::spinOnce();
    Quaternion q ;

    current_pose.x = pose_message->pose.pose.position.x;
    current_pose.y = pose_message->pose.pose.position.y;

    q.w = pose_message->pose.pose.orientation.w;
    q.x = pose_message->pose.pose.orientation.x;
    q.y = pose_message->pose.pose.orientation.y;
    q.z = pose_message->pose.pose.orientation.z;

    EulerAngles angles = ToEulerAngles(q);
    current_pose.th = angles.yaw;

    // std::string s;
    // std::stringstream sstm;
    // sstm << "x=" << current_pose.x << ", y=" << current_pose.y << ", theta=" << current_pose.th << "\n";
    // pose_info.data = sstm.str();
	// pose_publisher.publish(pose_info);    
}

void go_to_goal(PoseTurtlebot3 goal){
    ros::Rate loop_rate(10);

    ROS_INFO_STREAM("START VALUES START: " );
    ROS_INFO_STREAM("start_x=" << current_pose.x << ", start_y=" << current_pose.y << ", start_th=" << current_pose.th);

    double e_distance = abs( sqrt( pow( current_pose.x - goal.x, 2 ) + pow( current_pose.y - goal.y, 2 ) ) );
    double e_angle_alpha = put_angle_in_range(atan2((goal.y - current_pose.y), (goal.x - current_pose.x)) - current_pose.th);
    double e_angle_beta = put_angle_in_range( goal.th -current_pose.th -e_angle_alpha );


    ROS_INFO_STREAM("e_distance[" << e_distance << "] = abs( sqrt( pow( current_pose.x[" << current_pose.x << "] - goal.x[ " << goal.x << 
    "], 2 ) + pow( current_pose.y[" << current_pose.y << "] - goal.y[" << goal.y << "], 2 ) ) );");
    ROS_INFO_STREAM("e_alpha[" << e_angle_alpha << "] = atan2((goal.y[" << goal.y << "] - current_pose.y[" << current_pose.y << "]), (goal.x[" << goal.x << "] - current_pose.x[" 
    << current_pose.x << "])) - current_pose.th[" << current_pose.th << "]");
    ROS_INFO_STREAM("e_angle_beta[" << e_angle_beta << "] = goal_th[" << goal.th << "] - current_pose.th[" << current_pose.th << "] - e_angle_alpha[" << e_angle_alpha << "]");
    ROS_INFO_STREAM("START VALUES END" << std::endl);
    // ros::Rate loop_rate(10);
    sleep(1);


    //while (e_distance > 0.05){
    ROS_INFO_STREAM ("e_angle_alpha=" << abs(e_angle_alpha) << "> deg2rad(5)=" << deg2rad(5) );

    // while (abs(e_angle_alpha) > deg2rad(2)){
    while (e_distance > 0.05 ){
    // while (e_distance > 0.04 || abs(e_angle_alpha) > deg2rad(5) || abs(e_angle_beta) > deg2rad(5)){
        e_distance = abs( sqrt( pow( current_pose.x - goal.x, 2 ) + pow( current_pose.y - goal.y, 2 ) ) );
        e_angle_alpha = put_angle_in_range(atan2((goal.y - current_pose.y), (goal.x - current_pose.x)) - current_pose.th);
        e_angle_beta = put_angle_in_range( goal.th - current_pose.th -e_angle_alpha);
        vel_msg.linear.x = k_linear * e_distance;
        // vel_msg.angular.z = k_alpha * e_angle_alpha;
        // vel_msg.angular.z = k_beta * e_angle_beta;
        vel_msg.angular.z = k_alpha * e_angle_alpha + k_beta * e_angle_beta;

        ROS_INFO_STREAM("e_distance[" << e_distance << "] = abs( sqrt( pow( current_pose.x[" << current_pose.x << "] - goal.x[ " << goal.x << 
        "], 2 ) + pow( current_pose.y[" << current_pose.y << "] - goal.y[" << goal.y << "], 2 ) ) );");
        ROS_INFO_STREAM("e_alpha[" << e_angle_alpha << "] = atan2((goal.y[" << goal.y << "] - current_pose.y[" << current_pose.y << "]), (goal.x[" << goal.x << "] - current_pose.x[" 
        << current_pose.x << "])) - current_pose.th[" << current_pose.th << "]");
        ROS_INFO_STREAM("e_angle_beta[" << e_angle_beta << "] = goal.th[" << goal.th << "] - current_pose.th[" << current_pose.th << "] - e_angle_alpha[" << e_angle_alpha << "]");
        
        ROS_INFO_STREAM("vel_msg.linear.x =" << vel_msg.linear.x);
        ROS_INFO_STREAM("vel_msg.angular.z =" << vel_msg.angular.z << std::endl);
        publisher.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    endMovement();
}


void endMovement(){
    ROS_INFO_STREAM("Movement finished, now it will be set to 0!" << std::endl << std::endl);

    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    publisher.publish(vel_msg);
    ros::spinOnce();
}

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

double rad2deg(double rad){
    return rad * 180 / PI;
}

double deg2rad(double deg){
    return deg * PI / 180;
}

double put_angle_in_range(double angle){
    if (angle > PI){
        ROS_INFO_STREAM("angle > PI: angle=" << angle);
        return angle - 2 * PI;
    }else if (angle < -PI){
        ROS_INFO_STREAM("angle < -PI: angle=" << angle);
        return angle + 2 * PI;
    }else {
        return angle;
    }
}