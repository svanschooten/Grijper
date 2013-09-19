#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>

void gripperCallback(const std_msgs::Int32::ConstPtr&);
ros::Publisher publisher;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripper_controller");
	ros::NodeHandle n;
	ros::Publisher subscriber = n.subscribe("phidget_values", 1000, gripperCallback);
	publisher = n.advertise<std_msgs::Int32>("gripper_force", 1000);
	ros:spin();

	return 0;
}

void gripperCallback(const std_msgs::Int32::ConstPtr& msg){
	ROS_INFO("received %i", msg->data);
}