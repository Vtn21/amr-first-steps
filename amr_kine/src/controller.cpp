#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>

#define B 0.35
#define R 0.1

double calc_wL(double v, double w) {
	double wL = (v - B*w/2) / R;
	return wL;
}

double calc_wR(double v, double w) {
	double wR = (v + B*w/2) / R;
	return wR;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "kine_controller");
    	ros::NodeHandle node;

	ros::Publisher pubL = node.advertise<std_msgs::Float64>("vrep/vehicle/motorLeftSpeed", 1);
	ros::Publisher pubR = node.advertise<std_msgs::Float64>("vrep/vehicle/motorRightSpeed", 1);

	ros::Rate loop_rate(10);

	while(ros::ok()) {
		ROS_INFO("Type values for linear and angular speed:");
		double v, w;
		std::cin >> v >> w;
		std_msgs::Float64 wL, wR;
		wL.data = calc_wL(v, w);
		wR.data = calc_wR(v, w);
		pubL.publish(wL);
		pubR.publish(wR);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
