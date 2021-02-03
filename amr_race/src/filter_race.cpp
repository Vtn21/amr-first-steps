#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;
geometry_msgs::Twist carrot_twist, obstacle_twist;
bool obs_bool = false;

void carrot_cb(const geometry_msgs::TwistConstPtr &msg) {
    carrot_twist = *msg;
    if(obs_bool) {
        pub.publish(obstacle_twist);
    }
    else {
        pub.publish(carrot_twist);
    }
}

void bool_cb(const std_msgs::BoolConstPtr &msg) {
    obs_bool = msg->data;
}

void twist_cb(const geometry_msgs::TwistConstPtr &msg) {
    obstacle_twist = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xupacaaso_filter");
    ros::NodeHandle node;
    
    pub = node.advertise<geometry_msgs::Twist>("xupacaaso/cmd_vel", 1);
    
    ros::Subscriber sub_carrot = node.subscribe("xupacaaso/filter_vel", 1, carrot_cb);
    ros::Subscriber sub_obs_bool = node.subscribe("xupacaaso/obstacle/bool", 1, bool_cb);
    ros::Subscriber sub_obs_twist = node.subscribe("xupacaaso/obstacle/twist", 1, twist_cb);
    
    ros::spin();
}