#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

geometry_msgs::Twist twist_mux_vel;
nav_msgs::Odometry odom;
bool stopped = false;

void cmdCallback(const geometry_msgs::Twist &msg) {
    if (msg.linear.x == 0 && msg.angular.z == 0)
        stopped = true;
    else
        stopped = false;
    twist_mux_vel = msg;
}

void odomCallback(const nav_msgs::Odometry &msg) {
    odom = msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "active_compliance");
    ros::NodeHandle *nh = new ros::NodeHandle;

    ros::Subscriber odom_subscriber = nh->subscribe("odometry/base_raw", 1, odomCallback);
    ros::Subscriber twist_mux_vel_subscriber = nh->subscribe("twist_mux/cmd_vel", 1, cmdCallback);
    ros::Publisher push_detected_publisher = nh->advertise<std_msgs::Bool>("active_compliance/push_detected", 1);
    ros::Publisher active_compliance_status_publisher = nh->advertise<std_msgs::Bool>("active_compliance/status", 1);

    ros::Rate r(30); // 30 hz

    std_msgs::Bool push_detected, status;
    bool last_status = false;

    ROS_INFO("active compliance node started");

    while(ros::ok()) {
        status.data = false;
            if (fabs(twist_mux_vel.linear.x - odom.twist.twist.linear.x) > 0.001 && stopped == true){
                status.data = true;
                if (status.data != last_status) {
                    push_detected.data = true;
                    push_detected_publisher.publish(push_detected);
                }
            }
        last_status = status.data;
        active_compliance_status_publisher.publish(status);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
