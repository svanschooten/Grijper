#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <Grijper/gripper_cmd.h>
#include <sstream>
#include <stdio.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/CDynamixelROS.h>

using std::cout;
using std::endl;
using std::hex;
using std::dec;

void gripperControl(const Grijper::gripper_cmd::ConstPtr&);
ros::Publisher publisher;
bool execute(ArgList);
bool open();
bool close(float);

#define DXLC_SAFE_CALL(call) \
  do { \
    int ret = call; \
    if (ret != DXL_SUCCESS) { \
      cout << "Error:" << endl << "  " << C3mxl::translateErrorCode(ret) << " (0x" << hex << ret << dec << ")" << endl; \
    } \
  } while (0)

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripper_actuator");
	ros::NodeHandle n;
	ros::Publisher controller = n.subscribe("gripper_controll", 1000, gripperControl);
	gripper_state = n.advertise<std_msgs::Float32>("gripper_state", 1000);
	ros:spin();

	return 0;
}

void gripperControl(const Grijper::gripper_cmd::ConstPtr& msg){
	String cmd = msg->cmd.c_str();
	float force = msg->force;
	float current = calc_current(force);
	ROS_INFO("Actuating gripper: %sing %fN -> %fA",cmd, force, current);
}

bool execute(ArgList args){
	return true;
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