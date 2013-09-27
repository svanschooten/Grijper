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
bool close();
float calc_current(float);
float current = 0.04;
CDxlGeneric *motor_;
LxSerial serial_port_;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripper_actuator");
	CDxlConfig *config = new CDxlConfig();
	ROS_INFO("Using direct connection");
	motor_ = new C3mxl();

	serial_port_.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
	serial_port_.set_speed(LxSerial::S921600);
	motor_->setSerialPort(&serial_port_);

	motor_->setConfig(config->setID(109));
	motor_->init(false);
	motor_->set3MxlMode(CURRENT_MODE);

	delete config;

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
	current = calc_current(force);
	ROS_INFO("Actuating gripper: %sing %fN -> %fA",c_cmd, force, current);
	if(cmd.compare("open") == 0){
		open();
	}else if(cmd.compare("close") == 0){
		close();
	}else{
		ROS_INFO("Invalid command supplied");
	}
	
}

bool open(){
	motor_->setCurrent(-1*current);
	return true;
}

bool close(){
	motor_->setCurrent(current);
	return true;
}

float calc_current(float force){
	return 0.04;
}