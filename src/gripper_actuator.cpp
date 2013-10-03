#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Grijper/command.h>
#include <threemxl/C3mxlROS.h>

using namespace std;

void gripperControl(const Grijper::command::ConstPtr&);
ros::Publisher gripper_state; /*!< Gripper state publisher. */
ros::Publisher motor_current;
void shutdown(const std_msgs::Bool::ConstPtr&);
bool open();
bool close();
bool relax();
float calc_current(float);
float current = 0.04;  /*!< Target current variable, default is set to 0.04. */
CDxlGeneric *motor_;  /*!< Threemxl generic motor controller pointer. */

/*! \brief Main method of this node. All commands are evaluated here, and motor actuation is controlled from here.

This method controls the motor powering the gripper. This also meand publishing the gripper state when it has changed.

This node also listens to the shutdown command given by the controller.

*/
int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripper_actuator");
	CDxlConfig *config = new CDxlConfig();
	ROS_INFO("Using direct connection");
	motor_ = new C3mxl();
	LxSerial serial_port_;
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
	motor_current = n.advertise<std_msgs::Float32>("motor_current", 1);
	ros::Subscriber sd = n.subscribe("shutdown", 1, shutdown);
	ros::spin();

	return 0;
}

/*! \brief This method controls the motor.

This method is called every time a command from the controller has been given.
This command is evaluated and executed.
The msg parameter contains the 'cmd' string describing the command to be executed and also a 'force' float that indicates the target force.
This target force is converted to a current, which is passed in to the motor, with a sign and magnitude according to the command.

\param msg The message containing the command and target force.
*/
void gripperControl(const Grijper::command::ConstPtr& msg){
	string cmd = msg->cmd;
	const char* c_cmd = cmd.c_str();
	float force = msg->force;
	current = calc_current(force);	
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

float getCurrent(){
	motor_->getState();
	return motor_->getPresentCurrent();
}

/*! \brief Small controlling method for opening the gripper.

This method opens the gripper with the corresponding force.
\return True
*/
bool open(){
	ROS_INFO("Opening gripper");
	motor_->setCurrent(current);
	return true;
}

/*! \brief Small controlling method for closing the gripper.

This method closes the gripper with the corresponding force.
\return True
*/
bool close(){
	ROS_INFO("Closing gripper");
	motor_->setCurrent(-1*current);
	return true;
}

/*! \brief Small controlling method for relaxing the gripper.

This method relaxed the gripper.
\return True
*/
bool relax(){
	ROS_INFO("Relaxing gripper");
	motor_->setCurrent(0);
	return true;
}

/*! \brief Conversion method to convert a target force into a current.

Converts a target force into a current that can be passed on to the motor.
\param force The target force as a float.
\return The calculated current.
*/
float calc_current(float force){
	return force;
}

/*! \brief Controll method to shutdown this ROS node when the command is given.

This method will shutdown this node when the command is given by the console.
\param b The message containing the command. Nothing is done with the command since we know what it will be when this message is received.
*/
void shutdown(const std_msgs::Bool::ConstPtr& b){
	relax();
	ros::shutdown();
}
