#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

void callback(const nav_msgs::OdometryConstPtr &msg) {
	geometry_msgs::Quaternion qt;
	qt = msg->pose.pose.orientation;
	ROS_INFO("Yaw: %lf degrees", tf::getYaw(qt) * 180/M_PI);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "quat2yaw");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("vrep/vehicle/odometry", 1, callback);
	ros::spin();	
}
