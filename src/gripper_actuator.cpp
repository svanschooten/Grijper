#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Grijper/command.h>
#include <threemxl/C3mxlROS.h>

using namespace std;

void gripperControl(const Grijper::command::ConstPtr&);
ros::Publisher gripper_state;
ros::ServiceClient client;
void shutdown(const std_msgs::Bool::ConstPtr&);
bool open();
bool close();
bool relax();
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
	ros::Subscriber sd = n.subscribe("shutdown", 1, shutdown);
	ros::spin();

	return 0;
}

void gripperControl(const Grijper::command::ConstPtr& msg){
	string cmd = msg->cmd;
	const char* c_cmd = cmd.c_str();
	float force = msg->force;
	current = force;	
	ROS_INFO("Actuating gripper: %sing %fN -> %fA",c_cmd, force, current);
	if(cmd.compare("open") == 0){
		open();
	}else if(cmd.compare("relax") == 0){
		relax();
	}else if(cmd.compare("close") == 0){
		close();
	}else{
		ROS_INFO("Invalid command supplied");
	}
}

bool open(){
	ROS_INFO("Opening gripper");
	motor_->setCurrent(current);
	return true;
}

bool close(){
	ROS_INFO("Closing gripper");
	motor_->setCurrent(-1*current);
	return true;
}

bool relax(){
	ROS_INFO("Relaxing gripper");
	motor_->setCurrent(0);
	return true;
}

float calc_current(float force){
	return force;
}

void shutdown(const std_msgs::Bool::ConstPtr& b){
	relax();
	ros::shutdown();
}