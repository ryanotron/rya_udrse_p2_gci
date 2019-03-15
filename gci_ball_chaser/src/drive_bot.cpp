#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_cmd_pub;

double lin_x = 0.0;
double ang_z = 0.0;


bool drive_to_target_srv_handler(ball_chaser::DriveToTarget::Request& req,
                                 ball_chaser::DriveToTarget::Response& res) {
    lin_x = req.linear_x;
    ang_z = req.angular_z;
    
    res.linear_x = lin_x;
    res.angular_z = ang_z;
    
    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle nh;
    
    motor_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::ServiceServer drive_to_target_srv = nh.advertiseService("/ball_chaser/command_robot", drive_to_target_srv_handler);
    
    geometry_msgs::Twist motor_cmd;
    
    while (ros::ok()) {
        ros::spinOnce();
        
        motor_cmd.linear.x = lin_x;
        motor_cmd.linear.y = 0.0;
        motor_cmd.linear.z = 0.0;
        motor_cmd.angular.x = 0.0;
        motor_cmd.angular.y = 0.0;
        motor_cmd.angular.z = ang_z;
        
        motor_cmd_pub.publish(motor_cmd);
        
    }
    
    return 0;
}