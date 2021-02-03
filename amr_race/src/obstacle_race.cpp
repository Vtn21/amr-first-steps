#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

#define V_LINEAR 1;
#define V_ANGULAR 1;

#define RANGE_FRONT 0.4
#define RANGE_SIDE 0.3

ros::Publisher pub_twist, pub_bool;
std_msgs::Bool msg_bool;
geometry_msgs::Twist msg_twist;
float sonarFront, sonarLeft, sonarRight;

bool check_detection_side(float variable){
	if(variable!=0 && variable<RANGE_SIDE){
		return true;
	}else{
		return false;
	}
	
}

bool check_detection_front(float variable){
	if(variable!=0 && variable<RANGE_FRONT){
		return true;
	}else{
		return false;
	}
}

void front_cb(const std_msgs::Float32ConstPtr &msg) {
	ROS_INFO("Front: %g\t Left: %g\t Right: %g", sonarFront, sonarLeft, sonarRight);
	sonarFront = msg->data;

	// Logic 1
	/*
   if(sonarFront != 0) {
	   if(sonarLeft != 0 && sonarRight != 0) {
		   msg_bool.data = false;
   }else{
		   msg_bool.data = true;
		   if(sonarRight != 0) {
			   msg_twist.linear.x = 0;
			   msg_twist.angular.z = V_ANGULAR;
		   }
		   else{
			   msg_twist.linear.x = 0;
			   msg_twist.angular.z = -V_ANGULAR;
		   }
	   }
   }else{
	   msg_bool.data = false;  
   } */

	// Logic 2
	if(sonarFront==0 && sonarRight==0 && sonarLeft==0){
		msg_bool.data = false;
	}else{
		if(check_detection_side(sonarLeft)){
			msg_bool.data = true;
			if(check_detection_front(sonarFront)){
				msg_twist.linear.x = 0;
			}else{
				msg_twist.linear.x = V_LINEAR;
			}
			msg_twist.angular.z = -V_ANGULAR;    
		}
	
		if(check_detection_side(sonarRight)){
			msg_bool.data = true;
			if(check_detection_front(sonarFront)){
				msg_twist.linear.x = 0;
			}else{
				msg_twist.linear.x = V_LINEAR;
			}
			msg_twist.angular.z = V_ANGULAR;
		} 
		
		if(check_detection_front(sonarFront)){
			msg_bool.data = true;
			
			msg_twist.linear.x = 0;
			
			if(check_detection_side(sonarLeft) && check_detection_side(sonarRight)){ 
				if(sonarLeft<sonarRight){
					msg_twist.angular.z = -V_ANGULAR;  
				}else{
					msg_twist.angular.z = V_ANGULAR;
				}
			}else if(check_detection_side(sonarLeft)){
					msg_twist.angular.z = -V_ANGULAR;
			}else{
					msg_twist.angular.z = V_ANGULAR;
			}            
		}
		
		if(check_detection_side(sonarLeft) && check_detection_side(sonarRight) && sonarFront<0.2){ // Reverse Case
			msg_bool.data = true;
			msg_twist.linear.x = -5*V_LINEAR;
			msg_twist.angular.z = 0; 
	/*
			if(sonarLeft<sonarRight){
				msg_twist.angular.z = -V_ANGULAR;  
			}else{
				msg_twist.angular.z = V_ANGULAR;
			}
		}
		if(sonarLeft != 0 && sonarRight != 0 && sonarFront == 0) { //Only lateral sensors (both)
			msg_bool.data = true;
			msg_twist.linear.x = V_LINEAR;
			msg_twist.angular.z = 0;
		} */
	}

	//Logic 3
	/*
	if(sonarFront == 0 && sonarLeft == 0 && sonarRight == 0) {
		msg_bool.data = false;
	}
	else {
		msg_bool.data = true;
		if(sonarFront != 0) {
			if(check_detection_side(sonarLeft)) {
				msg_twist.angular.z = -V_ANGULAR;
				msg_twist.linear.x = 0;
			}
			else if(check_detection_side(sonarRight)) {
				msg_twist.angular.z = V_ANGULAR;
				msg_twist.linear.x = 0;
			}
			else {
				msg_twist.angular.z = V_ANGULAR;
				msg_twist.linear.x = V_LINEAR;
			}
		}
		else {
			if(sonarLeft != 0 && sonarRight != 0) {
				if(abs(sonarLeft - sonarRight) < 0.05) {
					msg_twist.angular.z = 0;
					msg_twist.linear.x = -2*V_LINEAR;
				}
				else {
					msg_twist.angular.z = 0;
					msg_twist.linear.x = V_LINEAR;
				}
			}
			else if(sonarLeft != 0) {
				msg_twist.angular.z = -V_ANGULAR;
				msg_twist.linear.x = V_LINEAR;
			}
			else {
				msg_twist.angular.z = V_ANGULAR;
				msg_twist.linear.x = V_LINEAR;
			}
		} */
	} 
	pub_bool.publish(msg_bool);
	pub_twist.publish(msg_twist);
}

void left_cb(const std_msgs::Float32ConstPtr &msg) {
	sonarLeft = msg->data;
}

void right_cb(const std_msgs::Float32ConstPtr &msg) {
	sonarRight = msg->data;
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "xupacaaso_obstacle");
	ros::NodeHandle node;
	
	msg_bool.data = false;
	
	pub_twist = node.advertise<geometry_msgs::Twist>("xupacaaso/obstacle/twist", 1);
	pub_bool = node.advertise<std_msgs::Bool>("xupacaaso/obstacle/bool", 1);
	
	ros::Subscriber sub_front = node.subscribe("vrep/redracer/frontSonar", 1, front_cb);
	ros::Subscriber sub_left = node.subscribe("vrep/redracer/sonarLeft", 1, left_cb);
	ros::Subscriber sub_right = node.subscribe("vrep/redracer/sonarRight", 1, right_cb);
	
	ros::spin();
}