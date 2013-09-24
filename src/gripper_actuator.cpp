#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sstream>
#include <stdio.h>
#include <readline/readline.h>
#include <readline/history.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/CDynamixelROS.h>

using std::cout;
using std::endl;
using std::hex;
using std::dec;

void gripperActuate(const std_msgs::Float32::ConstPtr&);
ros::Publisher publisher;
bool execute(ArgList);
bool open();
bool close(double);

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
	ros::Publisher subscriber = n.subscribe("gripper_force", 1000, gripperActuate);
	publisher = n.advertise<std_msgs::Float32>("gripper_values", 1000);
	ros:spin();

	return 0;
}

void gripperActuate(const std_msgs::Float32::ConstPtr& msg){
	float data = msg->data;
	ROS_INFO("actuating: %f", data);
}

bool execute(ArgList args){
	return true;
}

bool open(){
	return true;
}

bool close(float force){
	return true;
}