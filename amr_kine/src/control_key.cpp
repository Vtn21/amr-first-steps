//For use together with teleop_twist_keyboard.py

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#define B 0.35
#define R 0.1

ros::Publisher pubL, pubR;

double calc_wL(double v, double w) {
	double wL = (v - B*w/2) / R;
	return wL;
}

double calc_wR(double v, double w) {
	double wR = (v + B*w/2) / R;
	return wR;
}

void callback(const geometry_msgs::TwistConstPtr &msg) {
	std_msgs::Float64 wL, wR;
	wL.data = calc_wL(msg->linear.x, msg->angular.z);
	wR.data = calc_wR(msg->linear.x, msg->angular.z);
	pubL.publish(wL);
	pubR.publish(wR);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "kine_controller");
    	ros::NodeHandle node;

	pubL = node.advertise<std_msgs::Float64>("vrep/vehicle/motorLeftSpeed", 1);
	pubR = node.advertise<std_msgs::Float64>("vrep/vehicle/motorRightSpeed", 1);

	ros::Subscriber sub = node.subscribe("cmd_vel", 1, callback);

	ros::spin();
}
