#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <cmath>

#define XI 4
#define YI 1.5
#define XF -3
#define YF 1.5
#define K 1
#define DELTA 0.5

double x[4] = {4, -3, -3, 4}, y[4] = {1.5, 1.5, -1.5, -1.5};
int i = 0;

/* Colocar um vetor com quatro pontos de waypoint (retângulo) e, a cada vez que atingir um, trocar para o outro.
O robô deve ficar seguindo o loop.
*/

ros::Publisher pub;
geometry_msgs::Twist twist;

double theta_u, theta, beta, r_u, r, x_c, y_c;

double normalize_angle(double angle) {
	if(angle > M_PI) return(angle - 2*M_PI);
	else if(angle < -M_PI) return(angle + 2*M_PI);
	else return angle;
}

bool accomplished(const nav_msgs::OdometryConstPtr &od) {
	if(std::sqrt(std::pow(od->pose.pose.position.x - x[(i + 1) % 4], 2) + std::pow(od->pose.pose.position.y - y[(i + 1) % 4], 2)) < 0.1) return true;
	else return false;
}

double calc_orientation_error(const nav_msgs::OdometryConstPtr &od, double x_c, double y_c) {
	return normalize_angle(std::atan2(y_c - od->pose.pose.position.y, x_c - od->pose.pose.position.x) - tf::getYaw(od->pose.pose.orientation));
}

void callback(const nav_msgs::OdometryConstPtr &msg) {
	if(accomplished(msg)) {
		twist.linear.x = 0;
		twist.angular.z = 0;
		ROS_INFO("Target reached.");
		i = (i + 1) % 4;
	}
	else {
		theta_u = normalize_angle(std::atan2(y[i] - msg->pose.pose.position.y, x[i] - msg->pose.pose.position.x));
		theta = normalize_angle(std::atan2(y[(i + 1) % 4] - y[i], x[(i + 1) % 4] - x[i]));
		beta = normalize_angle(theta - theta_u);
		r_u = std::sqrt(std::pow(msg->pose.pose.position.y - y[i], 2) + std::pow(msg->pose.pose.position.x - x[i], 2));
		r = r_u * std::sqrt(1 - std::pow(std::sin(beta), 2));
		x_c = (r + DELTA) * std::cos(theta) + x[i];
		y_c = (r + DELTA) * std::sin(theta) + y[i];
		twist.angular.z = K * calc_orientation_error(msg, x_c, y_c);
		twist.linear.x = 0.2;
	}
	pub.publish(twist);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "carrot");
	ros::NodeHandle node;
	pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Subscriber sub = node.subscribe("vrep/vehicle/odometry", 1, callback);
	ros::spin();
}