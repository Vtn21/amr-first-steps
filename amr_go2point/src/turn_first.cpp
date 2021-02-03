#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <cmath>

#define TARGET_X 4
#define TARGET_Y 1.5
#define K 0.5

ros::Publisher pub;
geometry_msgs::Twist twist;

double normalize_angle(double angle) {
	if(angle > M_PI) return(angle - 2*M_PI);
	else if(angle < -M_PI) return(angle + 2*M_PI);
	else return angle;
}

bool accomplished(const nav_msgs::OdometryConstPtr &od) {
	if(std::sqrt(std::pow(od->pose.pose.position.x - TARGET_X, 2) + std::pow(od->pose.pose.position.y - TARGET_Y, 2)) < 0.1) return true;
	else return false;
}

double calc_orientation_error(const nav_msgs::OdometryConstPtr &od) {
	return normalize_angle(std::atan2(TARGET_Y - od->pose.pose.position.y, TARGET_X - od->pose.pose.position.x) - tf::getYaw(od->pose.pose.orientation));
}

void callback(const nav_msgs::OdometryConstPtr &msg) {
	if(calc_orientation_error(msg) > 0.1) twist.linear.x = 0;
    else twist.linear.x = 0.2;
    if(accomplished(msg)) {
        twist.angular.z = 0;
        twist.linear.x = 0;
        ROS_INFO("Target reached.");
    }
    else twist.angular.z = K * calc_orientation_error(msg);
	pub.publish(twist);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "quat2yaw");
	ros::NodeHandle node;
	pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Subscriber sub = node.subscribe("vrep/vehicle/odometry", 1, callback);
	ros::spin();
}
