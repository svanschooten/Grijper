#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <Grijper/command.h>

using namespace std;

void setForce(const std_msgs::Float32::ConstPtr&);
void gripperPhidget(const std_msgs::Float32::ConstPtr&);
void gripperCommand(const std_msgs::String::ConstPtr&);
ros::Publisher control;
bool close_gripper(int);
bool gripper_open = false;
float force = 0.5;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripper_controller");
	ros::NodeHandle n;
	ros::Subscriber force_sub = n.subscribe("gripper_force", 1000, setForce);
	ros::Subscriber command_sub = n.subscribe("command", 1000, gripperCommand);
	ros::Subscriber phidget_sub = n.subscribe("phidget_value", 1000, gripperPhidget);
	control = n.advertise<Grijper::command>("gripper_controll", 1000);
	ros::spin();

	return 0;
}

void gripperPhidget(const std_msgs::Float32::ConstPtr& msg){
	int sensor_value = msg->data;
	ROS_INFO("received sensor value: %i", sensor_value);
	if(close_gripper(sensor_value) && gripper_open){
		Grijper::command msg;
		msg.cmd = "close";
		msg.force = force;
		gripper_open = false;
		control.publish(msg);
	}
}

void setForce(const std_msgs::Float32::ConstPtr& msg){
	force = msg->data;	
	ROS_INFO("Setting controller force to: %f\n", force);
}

void gripperCommand(const std_msgs::String::ConstPtr& msg){
	string cmd = msg->data;
	Grijper::command ctl;
	if(cmd.compare("open") == 0){
		ctl.cmd = "open";
		ctl.force = force;
		gripper_open = true;
		control.publish(ctl);
	} else  if(cmd.compare("close") == 0){
		ctl.cmd = "close";
		ctl.force = force;
		gripper_open = false;
		control.publish(ctl);
	} else {
		ROS_INFO("Received invalid command: %s\n", cmd.c_str());
	}
}

bool close_gripper(int distance){
	return distance < 100; //TODO dit nog bewerken afhankelijk van de sensor
}