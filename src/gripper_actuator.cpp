#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <Grijper/command.h>
#include <threemxl/C3mxlROS.h>

using namespace std;

void gripperControl(const Grijper::command::ConstPtr&);
ros::Publisher gripper_state;
bool open();
bool close(float);
float calc_current(float);

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripper_actuator");
	ros::NodeHandle n;
	ros::Subscriber controller = n.subscribe("gripper_controll", 1000, gripperControl);
	gripper_state = n.advertise<std_msgs::Float32>("gripper_state", 1000);
	ros::spin();

	return 0;
}

void gripperControl(const Grijper::command::ConstPtr& msg){
	string cmd = msg->cmd;
	const char* c_cmd = cmd.c_str();
	float force = msg->force;
	float current = calc_current(force);
	ROS_INFO("Actuating gripper: %sing %fN -> %fA",c_cmd, force, current);
}

bool open(){
	return true;
}

bool close(float current){
	return true;
}

float calc_current(float force){
	return force;
}