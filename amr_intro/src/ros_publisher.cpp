#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_pub");
    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<std_msgs::String>("ros_world", 1);
    ros::Rate loopRate(10);
    std_msgs::String msg;
    
    while(ros::ok()) {
        msg.data = "Hello world!";
        ROS_INFO("Publishing...");
        pub.publish(msg);
        ROS_INFO("Done.");
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}