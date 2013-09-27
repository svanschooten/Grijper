#include <ros/ros.h>
#include <cmath>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <Grijper/command.h>

using namespace std;

void setForce(const std_msgs::Float32::ConstPtr&);
void gripperPhidget(const std_msgs::Int32::ConstPtr&);
void gripperCommand(const std_msgs::String::ConstPtr&);
ros::Publisher control;
bool close_gripper(int);
bool gripper_open = true;
float last_sensor_value = 0;
float force = 0.5;
int pause_time = 1500;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripper_controller");
	ros::NodeHandle n;
	ros::Subscriber force_sub = n.subscribe("gripper_force", 1, setForce);
	ros::Subscriber command_sub = n.subscribe("command", 1000, gripperCommand);
	ros::Subscriber phidget_sub = n.subscribe("phidget_value", 1, gripperPhidget);
	control = n.advertise<Grijper::command>("gripper_controll", 1);
	ros::spin();

	return 0;
}

void gripperPhidget(const std_msgs::Int32::ConstPtr& msg){

	int sensor_value = msg->data;
	
	if(abs(sensor_value - last_sensor_value) > 50){

		last_sensor_value = sensor_value;

		ROS_INFO("received sensor value: %i", sensor_value);
		if(close_gripper(sensor_value) && gripper_open){
			ROS_INFO("Closing gripper");
			Grijper::command msg;
			msg.cmd = "close";
			msg.force = force;
			gripper_open = false;
			control.publish(msg);
		}
		if(!close_gripper(sensor_value) && !gripper_open){
			ROS_INFO("Opening gripper");
			Grijper::command msg;
			msg.cmd = "open";
			msg.force = force;
			gripper_open = true;
			control.publish(msg);
		}
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
		ROS_INFO("Opening gripper");
		ctl.cmd = "open";
		ctl.force = force;
		gripper_open = true;
		control.publish(ctl);
	}else if(cmd.compare("relax") == 0){
		ROS_INFO("Relaxing gripper");
		ctl.cmd = "relax";
		ctl.force = 0;
		control.publish(ctl);
	}else if(cmd.compare("close") == 0){
		ROS_INFO("Closing gripper");
		ctl.cmd = "close";
		ctl.force = force;
		gripper_open = false;
		control.publish(ctl);
	}else{
		ROS_INFO("Received invalid command: %s\n", cmd.c_str());
	}
}

bool close_gripper(int distance){
	return distance < 100; //TODO dit nog bewerken afhankelijk van de sensor
}
