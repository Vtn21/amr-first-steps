#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;
geometry_msgs::Twist vel;

void callback(const std_msgs::Float32ConstPtr &msg) {
	if(msg->data == 0.0) vel.linear.x = 0.2;
	else vel.linear.x = 0.0;
	pub.publish(vel);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "sonar_handler");
	ros::NodeHandle node;
	pub = node.advertise<geometry_msgs::Twist>("my_vehicle_twist", 1);
	ros::Subscriber sub = node.subscribe("vrep/vehicle/frontSonar", 1, callback);
	ros::spin();
}
