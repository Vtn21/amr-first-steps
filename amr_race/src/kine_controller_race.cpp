#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#define MAX_ANGULAR_VELOCITY 15

// Global Variables
float v,w;
//float v=0.25,w=0;
float b=0.35,R=0.1; 

ros::Publisher pubL;
ros::Publisher pubR;

// Instantiate messages to be published
std_msgs::Float32 velAngularWheelL_msg;
std_msgs::Float32 velAngularWheelR_msg;

// Subscriber routine
// It's called everytime that there is a new message
void robotSpeedsCallback(const geometry_msgs::TwistPtr& msg) {
    double wL,wR,rate;
    
    //ROS_INFO("Received: Robot Speeds! Publishing motors linear velocities to Vrep...");
    
    v = msg->linear.x;
    w = msg->angular.z;
    
    // Check if ROS connection is fine each iteration
    
    // Build message according to a logic.
    wL = (v-(b/2.0)*w)/R;
    wR = (v+(b/2.0)*w)/R;
    
    if((wL>MAX_ANGULAR_VELOCITY) || (wR>MAX_ANGULAR_VELOCITY)){
        if(wL>=wR){
            double rate = wR/wL;
            
            wL = MAX_ANGULAR_VELOCITY;
            wR = rate*MAX_ANGULAR_VELOCITY;
        }else{
            double rate = wL/wR;
            
            wL = rate*MAX_ANGULAR_VELOCITY;
            wR = MAX_ANGULAR_VELOCITY;
        }
    }
    
    ROS_INFO("velAngularWheelL: %f\tvelAngularWheelR: %f",wL,wR);
        
    // Publish messages
    velAngularWheelL_msg.data = wL;
    velAngularWheelR_msg.data = wR;
    
    pubL.publish(velAngularWheelL_msg);
    pubR.publish(velAngularWheelR_msg);
}

int main(int argc, char **argv){    
    //ROS_INFO("Hello World using ROS function!");
    
    // Start ROS within the context of this node
    ros::init(argc, argv, "kine_pub");
    
    // Declare node.
    ros::NodeHandle node;
    
    // Declare topic to be published
    int queue_size = 1; 
    pubL = node.advertise<std_msgs::Float32>("/vrep/redracer/motorLeftSpeed",queue_size);
    pubR = node.advertise<std_msgs::Float32>("/vrep/redracer/motorRightSpeed",queue_size);
    
    // Declare topic to subscriber
    //ros::Subscriber sub = node.subscribe("/object_avoidance/robotSpeeds", queue_size, robotSpeedsCallback);
    //ros::Subscriber sub = node.subscribe("/cmd_vel", queue_size, robotSpeedsCallback);
    ros::Subscriber sub = node.subscribe("/xupacaaso/cmd_vel", queue_size, robotSpeedsCallback);
    
    
    // Define ROS loop rate (Hertz!)
    //int frequency = 60;
    //ros::Rate loop_rate(frequency); 
    ros::spin();
    
    return 0;
}