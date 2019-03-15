#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "ball_chaser/DriveToTarget.h"

ros::ServiceClient drive_to_target_clt;

float vx_max = 0.5000;
float wz_max = 1.5707;

float k_lin = vx_max/600.0;
float k_ang = wz_max/400.0;

void drive_robot(float lin_x, float ang_z) {
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    if (!drive_to_target_clt.call(srv)) {
        ROS_ERROR("Failed to call service: /ball_chaser/command_robot");
    }
}


void img_cb(const sensor_msgs::Image img) {
    int white_pixel = 255;
    
    int img_third = img.width/3;
    float lin_x, ang_z;
    
    int leftmost = img.width;
    int rightmost = 0;
    
    bool found = false;
    
//     ROS_INFO("size: %d, enc: %s", img.step*img.height, img.encoding.c_str());
    
    for (int i = 0; i < img.step*img.height; i+=3) {        
        bool white = img.data[i] == white_pixel;
        white = white && (img.data[i+1] == white_pixel);
        white = white && (img.data[i+2] == white_pixel);
        
        if (white) {
            found = true;
            int col = (i % img.step)/3;
            if (col < leftmost) {
                leftmost = col;
            }
            
            if (col > rightmost) {
                rightmost = col;
            }
        }
    }
    
    if (!found) {
        lin_x = 0.0;
        ang_z = 0.0;
    }
    else {
        int ball_centre = (leftmost + rightmost)/2;
        int ball_width = rightmost - leftmost;
        
//         ROS_INFO("c, w: %d, %d, third: %d, width: %d", ball_centre, ball_width, img_third, img.width);
        
        if ((ball_centre > img_third) && (ball_centre < 2*img_third)) {
            ang_z = 0.0;
            lin_x = k_lin * (600.0 - ball_width);
        }
        else {
            lin_x = 0.0;
            ang_z = k_ang * (400.0 - ball_centre);
        }
    }
    
    drive_robot(lin_x, ang_z);    
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "process_image");
    
    ros::NodeHandle nh;
    
    drive_to_target_clt = nh.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    ros::Subscriber img_sub = nh.subscribe("/camera/rgb/image_raw", 10, img_cb);
    
    ros::spin();
    
    return 0;
}