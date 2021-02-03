#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <cmath>

#define V_LINEAR 1.5
#define K 2
#define DELTA 0.6
#define TARGET_TOL 0.5

// WayPoints 1
//#define N_POINTS 17
//double x[] = {3.5750, 10.1500, 11.4000,  15.2250,  14.7000,   8.0750,  -5.6750, -13.8500, -13.3250, -11.2500,  -2.5000,  -1.6250,  -1.8000, -12.6500, -13.0000, -10.3250};
//double y[] = {0.0500,  0.0000, -9.5250, -14.2000, -15.6500, -19.9750, -20.0000, -14.9750, -13.1750, -12.6500, -12.7250, -12.5500, -11.0500, -10.7000,  -3.1250,  -0.2000};

// WayPoints 2
#define N_POINTS 13
double x[] = {3.5750, 10.1500, 11.4000,  14.9750,   8.0750,  -5.6750, -14.2000, -12.2500, -1.5500,  -1.4500, -12.6500, -13.0000, -10.2250};
double y[] = {0.0500,  0.0000, -9.5250, -15.3750, -19.9750, -20.0000, -14.1000, -12.8000,-12.5750, -11.0250, -10.7000,  -2.0750,  -0.0250};

int i = 0;

ros::Publisher pub;
geometry_msgs::Twist twist;

double theta_u, theta, beta, r_u, r, x_c, y_c;

double normalize_angle(double angle) {
	if(angle > M_PI) return(angle - 2*M_PI);
	else if(angle < -M_PI) return(angle + 2*M_PI);
	else return angle;
}

bool accomplished(const nav_msgs::OdometryConstPtr &od) {
	if(std::sqrt(std::pow(od->pose.pose.position.x - x[(i + 1) % N_POINTS], 2) + std::pow(od->pose.pose.position.y - y[(i + 1) % N_POINTS], 2)) < TARGET_TOL) return true;
	else return false;
}

double calc_orientation_error(const nav_msgs::OdometryConstPtr &od, double x_c, double y_c) {
	return normalize_angle(std::atan2(y_c - od->pose.pose.position.y, x_c - od->pose.pose.position.x) - tf::getYaw(od->pose.pose.orientation));
}

void callback(const nav_msgs::OdometryConstPtr &msg) {
	if(accomplished(msg)) {
		twist.linear.x = 0;
		twist.angular.z = 0;
		i = (i + 1) % N_POINTS;
		ROS_INFO("Target reached(%2.2f,%2.2f).Next target(%2.2f,%2.2f).",x[i],y[i],x[(i+1)%N_POINTS ],y[(i+1)%N_POINTS]);
	}
	else {
		theta_u = normalize_angle(std::atan2(y[i] - msg->pose.pose.position.y, x[i] - msg->pose.pose.position.x));
		theta = normalize_angle(std::atan2(y[(i + 1) % N_POINTS] - y[i], x[(i + 1) % N_POINTS] - x[i]));
		beta = normalize_angle(theta - theta_u);
		r_u = std::sqrt(std::pow(msg->pose.pose.position.y - y[i], 2) + std::pow(msg->pose.pose.position.x - x[i], 2));
		r = r_u * std::sqrt(1 - std::pow(std::sin(beta), 2));
		x_c = (r + DELTA) * std::cos(theta) + x[i];
		y_c = (r + DELTA) * std::sin(theta) + y[i];
		twist.angular.z = K * calc_orientation_error(msg, x_c, y_c);
		twist.linear.x = V_LINEAR;
	}
	pub.publish(twist);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "xupacaaso_carrot");
	ros::NodeHandle node;
	//pub = node.advertise<geometry_msgs::Twist>("/xupacaaso/cmd_vel", 1);
    pub = node.advertise<geometry_msgs::Twist>("/xupacaaso/filter_vel", 1);
	ros::Subscriber sub = node.subscribe("/vrep/redracer/odometry", 1, callback);
	ros::spin();
}