#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <Grijper/gripper_cmd.h>

void setForce(const std_msgs::Float32::ConstPtr&);
void gripperPhidget(const std_msgs::Float32::ConstPtr&);
void gripperCommand(const std_msgs::String::ConstPtr&);
ros::Publisher controll;
bool close_gripper(int);
bool gripper_open = false;
float force = 0.5;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripper_controller");
	ros::NodeHandle n;
	ros::Publisher force_sub = n.subscribe("gripper_force", 1000, setForce);
	ros::Publisher command_sub = n.subscribe("gripper_cmd", 1000, gripperCommand);
	ros::Publisher phidget_sub = n.subscribe("phidget_value", 1000, gripperPhidget);
	controll = n.advertise<Grijper::gripper_cmd>("gripper_controll", 1000);
	ros:spin();

	return 0;
}

void gripperPhidget(const std_msgs::Int32::ConstPtr& msg){
	int sensor_value = msg->data;
	ROS_INFO("received sensor value: %i", sensor_value);
	if(close_gripper(sensor_value) && gripper_open){
		Grijper::gripper_cmd msg;
		msg.cmd = "close";
		msg.force = force;
		gripper_open = false;
		controll.publish(msg);
	}
}

void setForce(const std_msgs::Float32::ConstPtr& msg){
	float force = msg->data;
	ROS_INFO("Setting controller force to: %f\n", force);
}

void gripperCommand(const std_msgs::String::ConstPtr& msg){
	String cmd = msg->data.c_str();
	switch(cmd){
		case "open":
			Grijper::gripper_cmd msg;
			msg.cmd = "open";
			msg.force = force;
			gripper_open = true;
			controll.publish(msg);
			break;
		case "close":
			Grijper::gripper_cmd msg;
			msg.cmd = "close";
			msg.force = force;
			gripper_open = true;
			controll.publish(msg);
			break;
		default:
			ROS_INFO("Received invalid command: %s\n", cmd);
			break;
	}
}

bool close_gripper(int distance){
	return distance < 10; //TODO dit nog bewerken afhankelijk van de sensor
}