#include <ros/ros.h>
#include <string>
#include <std_msgs/Int32.h>
//#include <rvix> en dan nog wat dingen

void rvizUpdater(const std_msgs::Int32::ConstPtr&);

int main(int argc, char **argv){
	
	ros::init(argc, argv, "rviz_object");
	ros::NodeHandle n;
	ros::Subscriber subscriber = n.subscribe("phidget_value", 1000, rvizUpdater);
	ros::spin();

	return 0;
}

void rvizUpdater(const std_msgs::Int32::ConstPtr& msg){
	ROS_INFO("rziv distance: %i", msg->data);
}