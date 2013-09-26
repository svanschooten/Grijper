#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>
//#include <rvix> en dan nog wat dingen

using namespace std;

void rvizUpdater(const std_msgs::Float32::ConstPtr&);

int main(int argc, char **argv){
	
	ros::init(argc, argv, "rviz_gripper");
	ros::NodeHandle n;
	ros::Subscriber subscriber = n.subscribe("gripper_state", 1000, rvizUpdater);
	ros::spin();

	return 0;
}

void rvizUpdater(const std_msgs::Float32::ConstPtr& msg){
	ROS_INFO("rziv distance: %f", msg->data);
}