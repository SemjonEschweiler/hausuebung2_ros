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
void dynamic_callback(hausuebung2_semjon_eschweiler::dynamicParamsConfig &config, uint32_t level);
void go_to_goal(double goal_x, double goal_y, double goal_th);
void endMovement();
double rad2deg(double rad);
double deg2rad(double deg);
double put_angle_in_range(double angle);
EulerAngles ToEulerAngles(Quaternion q);

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "tb3_nav_node");
	ros::NodeHandle n;

    dynamic_reconfigure::Server<hausuebung2_semjon_eschweiler::dynamicParamsConfig> server;
    dynamic_reconfigure::Server<hausuebung2_semjon_eschweiler::dynamicParamsConfig>::CallbackType f;

    f = boost::bind(&dynamic_callback, _1, _2);
    server.setCallback(f);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

	publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    subscriber_pose = n.subscribe("/odom", 10, pose_callback); //Why does pose_callback not get called sometimes?
    client = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");

    //Test your code here

	ROS_INFO("\n\n\n******START TESTING************\n");
	ros::Rate loop_rate(10);

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
}

void dynamic_callback(hausuebung2_semjon_eschweiler::dynamicParamsConfig &config, uint32_t level) {
    ROS_INFO_STREAM("Reconfigure Request: \n" << "\tk_linear: " << config.k_linear << std::endl <<
            "\tk_alpha: " << config.k_alpha << std::endl <<
            "\tk_beta: " << config.k_beta << std::endl <<
            "\tx_goal: " << config.x_goal << std::endl <<
            "\ty_goal: " << config.y_goal << std::endl <<
            "\tth_goal: " << config.th_goal << std::endl <<
            "\tstart_on_check: " << config.start_on_check << std::endl << std::endl);
    k_alpha = config.k_alpha;
    k_beta = config.k_beta;
    k_linear = config.k_linear;
    double th_goal = deg2rad(config.th_goal);
    if (checkbox_start_status_earlier == false && config.start_on_check == true){
        go_to_goal(config.x_goal, config.y_goal, th_goal);
    }
    if (checkbox_reset_status_earlier == false && config.reset_on_check == true){
        ROS_INFO("SERVICE IS CALLED TO RESET GAZEBO SIMULATION!!!");
        std_srvs::Empty srv;
        //send service
        if (client.call(srv)){

            ROS_INFO("Returned and reset!");

        }else {

            ROS_INFO("Return failed!");
        }
    }
    checkbox_start_status_earlier = config.start_on_check;
    checkbox_reset_status_earlier = config.reset_on_check;
}


void moveStraightLine(double distance, bool moveForward, double velocity){

    ROS_INFO_STREAM("Move in straight line!");
    double distanceMoved = 0;


    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate loop_rate(10);

    if (moveForward){
        vel_msg.linear.x = velocity;
    }else {
        vel_msg.linear.x = -velocity;
    }

    while (distanceMoved < distance){
        current_time = ros::Time::now();
        publisher.publish(vel_msg);
        ROS_INFO_STREAM("Distance moved: " << distanceMoved);
        ROS_INFO_STREAM("[Talker] I published a TwistMessage: \n" << vel_msg);
        distanceMoved += velocity * (current_time - last_time).toSec();
        ROS_INFO_STREAM("Distance moved (" << distanceMoved << ")= velocity (" << velocity << ") * (t1 (" << current_time.toSec() << ")-t0 (" << last_time.toSec() << "))(" << (current_time - last_time) << ")");
        ROS_INFO_STREAM("TIME is at the moment: " << ros::Time::now().toSec());
        
        ros::spinOnce();
        loop_rate.sleep();
        last_time = current_time;
    }
    ROS_INFO_STREAM("TIME is at the moment: " << ros::Time::now().toSec());
    ROS_INFO_STREAM("Distance moved: " << distanceMoved);
    endMovement();
}

void moveStraightLineOdom(double distance, bool moveForward, double velocity){
    ros::Rate loop_rate(100);

    double initial_x, initial_y, distance_traveled, distance_traveled_time_calc;
    ROS_INFO_STREAM("THIS IS WHAT CURRENT POSE IS: X=" << current_pose.x << ", Y= " << current_pose.y);
    
    initial_x = current_pose.x;
    initial_y = current_pose.y;
    while(initial_x == 0 && initial_y == 0){
        ROS_INFO_STREAM("Waiting for initials to receive proper values");
        initial_x = current_pose.x;
        initial_y = current_pose.y;
        ros::spinOnce();
        loop_rate.sleep();
    }
    distance_traveled_time_calc = 0;
    distance_traveled = sqrt(pow(current_pose.x - initial_x, 2) + pow(current_pose.y - initial_y, 2));
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ROS_INFO_STREAM("Move in straight line!");

    if (moveForward){
        vel_msg.linear.x = velocity;
    }else {
        vel_msg.linear.x = -velocity;
    }
    
    while (distance_traveled < distance){
        current_time = ros::Time::now();
        publisher.publish(vel_msg);
        ROS_INFO_STREAM("initial_x: " << initial_x << ", initial_y: " << initial_y);
        ROS_INFO_STREAM("current_x: " << current_pose.x << ", current_y: " << current_pose.y);


        distance_traveled = sqrt(pow(current_pose.x - initial_x, 2) + pow(current_pose.y - initial_y, 2));
        distance_traveled_time_calc += velocity * (current_time - last_time).toSec();
        ROS_INFO_STREAM("distance_traveled: " << distance_traveled);
        ROS_INFO_STREAM("distance_traveled_time_calc: " << distance_traveled_time_calc);


        ros::spinOnce();
        loop_rate.sleep();
        last_time = current_time;
    }
    endMovement();
}

void rotateByAngle(double angleDg, bool positiveRot, double angVelocityDg){
    double angle = angleDg * PI / 180;
    double angVelocity = angVelocityDg * PI / 180;
    ROS_INFO_STREAM("Move in straight line!");
    geometry_msgs::Twist vel_msg;
    double angleRotated = 0;
    double t0 = ros::Time::now().toSec();
    ros::Rate loop_rate(10);

    if (positiveRot){
        vel_msg.angular.z = angVelocity;
    }else {
        vel_msg.angular.z = -angVelocity;
    }

    while (angleRotated < angle){
        ROS_INFO_STREAM("[Talker] I published a TwistMessage: \n" << vel_msg);
        publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        angleRotated = angVelocity * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    }
    endMovement();
}


void go_to_goal(double goal_x, double goal_y, double goal_th){
    ros::Rate loop_rate(10);

    ROS_INFO_STREAM("START VALUES START: " );
    ROS_INFO_STREAM("start_x=" << current_pose.x << ", start_y=" << current_pose.y << ", start_th=" << current_pose.th);

    double e_distance = abs( sqrt( pow( current_pose.x - goal_x, 2 ) + pow( current_pose.y - goal_y, 2 ) ) );
    double e_angle_alpha = put_angle_in_range(atan2((goal_y - current_pose.y), (goal_x - current_pose.x)) - current_pose.th);
    double e_angle_beta = put_angle_in_range(goal_th - current_pose.th -e_angle_alpha);


    ROS_INFO_STREAM("e_distance[" << e_distance << "] = abs( sqrt( pow( current_pose.x[" << current_pose.x << "] - goal_x[ " << goal_x << 
    "], 2 ) + pow( current_pose.y[" << current_pose.y << "] - goal_y[" << goal_y << "], 2 ) ) );");
    ROS_INFO_STREAM("e_alpha[" << e_angle_alpha << "] = atan2((goal_y[" << goal_y << "] - current_pose.y[" << current_pose.y << "]), (goal_x[" << goal_x << "] - current_pose.x[" 
    << current_pose.x << "])) - current_pose.th[" << current_pose.th << "]");
    ROS_INFO_STREAM("e_angle_beta[" << e_angle_beta << "] = goal_th[" << goal_th << "] - current_pose.th[" << current_pose.th << "] - e_angle_alpha[" << e_angle_alpha << "]");
    ROS_INFO_STREAM("START VALUES END" << std::endl);
    // ros::Rate loop_rate(10);
    sleep(1);


    //while (e_distance > 0.05){
    ROS_INFO_STREAM ("e_angle_alpha=" << abs(e_angle_alpha) << "> deg2rad(5)=" << deg2rad(5) );

    // while (abs(e_angle_alpha) > deg2rad(2)){
    while (e_distance > 0.04 ){
    // while (e_distance > 0.04 || abs(e_angle_alpha) > deg2rad(5) || abs(e_angle_beta) > deg2rad(5)){
        e_distance = abs( sqrt( pow( current_pose.x - goal_x, 2 ) + pow( current_pose.y - goal_y, 2 ) ) );
        e_angle_alpha = put_angle_in_range(atan2((goal_y - current_pose.y), (goal_x - current_pose.x)) - current_pose.th);
        e_angle_beta = put_angle_in_range(goal_th - current_pose.th -e_angle_alpha);
        vel_msg.linear.x = k_linear * e_distance;
        // vel_msg.angular.z = k_alpha * e_angle_alpha;
        // vel_msg.angular.z = k_beta * e_angle_beta;
        vel_msg.angular.z = k_alpha * e_angle_alpha + k_beta * e_angle_beta;

        ROS_INFO_STREAM("e_distance[" << e_distance << "] = abs( sqrt( pow( current_pose.x[" << current_pose.x << "] - goal_x[ " << goal_x << 
        "], 2 ) + pow( current_pose.y[" << current_pose.y << "] - goal_y[" << goal_y << "], 2 ) ) );");
        ROS_INFO_STREAM("e_alpha[" << e_angle_alpha << "] = atan2((goal_y[" << goal_y << "] - current_pose.y[" << current_pose.y << "]), (goal_x[" << goal_x << "] - current_pose.x[" 
        << current_pose.x << "])) - current_pose.th[" << current_pose.th << "]");
        ROS_INFO_STREAM("e_angle_beta[" << e_angle_beta << "] = goal_th[" << goal_th << "] - current_pose.th[" << current_pose.th << "] - e_angle_alpha[" << e_angle_alpha << "]");
        
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