#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>

void gripperActuate(const std_msgs::Int32::ConstPtr&);
ros::Publisher publisher;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripper_actuator");
	ros::NodeHandle n;
	ros::Publisher subscriber = n.subscribe("gripper_force", 1000, gripperActuate);
	publisher = n.advertise<std_msgs::Int32>("gripper_values", 1000);
	ros:spin();

	return 0;
}

void gripperActuate(const std_msgs::Int32::ConstPtr& msg){
	int data = msg->data;
	ROS_INFO("actuating: %i", data);
}