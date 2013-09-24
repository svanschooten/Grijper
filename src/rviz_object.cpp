#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>

void rvizUpdater();

int main(int argc, char **argv){
	
	ros::init(argc, argv, "rviz_object");
	ros::NodeHandle n;
	ros::Publisher subscriber = n.subscribe("phidget_value", 1000, rvizUpdater);
	ros:spin();

	return 0;
}

void rvizUpdater(const std_msgs::Int32::ConstPtr& msg){
	ROS_INFO("rziv distance: %i", msg->data);
}