#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <string>

int counter = 0;

ros::Publisher pub;
std_msgs::Int32 msg_counter;

void callback(const std_msgs::StringConstPtr &msg) {
    ROS_INFO("Received: %s", msg->data.c_str());
	counter++;
	msg_counter.data = counter;
	pub.publish(msg_counter);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_sub");
    ros::NodeHandle node;
	pub = node.advertise<std_msgs::Int32>("ros_world/counter", 1);
    ros::Subscriber sub = node.subscribe("ros_world", 1, callback);
    ros::spin();
}
