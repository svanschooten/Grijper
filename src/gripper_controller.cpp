#include <ros/ros.h>
#include <cmath>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <Grijper/command.h>

using namespace std;

void setForce(const std_msgs::Float32::ConstPtr&);
void gripperPhidget(const std_msgs::Float32::ConstPtr&);
void gripperCommand(const std_msgs::String::ConstPtr&);
void shutdown(const std_msgs::Bool::ConstPtr&);
ros::Publisher control;  /*!< Controller message publisher. */
bool close_gripper(float);
bool open_gripper(float);
bool gripper_open = true;  /*!< Gripper state variable. */
bool force_open = false;  /*!< Force open state variable. */
float last_distance = 0;  /*!< Last sensor value, used for filtering and smoothing. */
float force = 0.35;  /*!< Force variable, is set by default to 0.35 and listens to the console. */

/*! \brief Main method of the controller node. All incoming messages are analysed and commands are sent from this node.

This node listens to commands given from the console, but also evaluates the values passed on from the phidget reader.
It evaluates the console commands and if needed passes them on to the actuator node.
Otherwise is extracts the distance from the sensor value and compares it to the threshold values to open (using open()) and close (using close()) the gripper.
If the command is received to alter the target force, that is set and used in future commands to the actuator.

This node also listens to the shutdown command from the console, and executes shutdown()

*/
int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripper_controller");
	ros::NodeHandle n;
	ros::Subscriber force_sub = n.subscribe("gripper_force", 1, setForce);
	ros::Subscriber command_sub = n.subscribe("command", 1000, gripperCommand);
	ros::Subscriber phidget_sub = n.subscribe("phidget_value", 1, gripperPhidget);
	ros::Subscriber sd = n.subscribe("shutdown", 1, shutdown);
	control = n.advertise<Grijper::command>("gripper_control", 1);
	ros::spin();

	return 0;
}

/*! \brief Method that evaluates the phidget sensor values and takes appropriate action.

This method is called every time the phidget passes on a value, extracting the distance (by using sensorToDistance(int)).
Once the distance has been calculated the value is compared to thresholds and a command is given to the actuator.

If the gripper was forced open, and the gripper should close due to proximity of an object, the force_open value is reset to false.

\param msg The message containing the Phidget sensor value.

*/
void gripperPhidget(const std_msgs::Float32::ConstPtr& msg){

	float distance = msg->data;
	
	if(abs(distance - last_distance) > 0.5){

		last_distance = distance;

		ROS_INFO("received distance: %f", distance);
		if(close_gripper(distance) && gripper_open && !force_open){
			ROS_INFO("Closing gripper");
			Grijper::command msg;
			msg.cmd = "close";
			msg.force = force;
			gripper_open = false;
			control.publish(msg);
		}
		if(open_gripper(distance) && !gripper_open){
			ROS_INFO("Opening gripper");
			Grijper::command msg;
			msg.cmd = "open";
			msg.force = force;
			gripper_open = true;
			control.publish(msg);
		}

		if(open_gripper(distance) && force_open){
			ROS_INFO("Resetting force_open");
			force_open = false;
		}
	}
}

/*! \brief Method for setting the force on message receiving

This method is called when the message for setting the force is received.
This new value is then set in force and will be used in future commands.

\param msg The message containing the new target force value.

*/
void setForce(const std_msgs::Float32::ConstPtr& msg){
	force = msg->data;	
	ROS_INFO("Setting controller force to: %f\n", force);
}

/*! \brief Method for evaluating and executing commands received from the console.

This method is called for every console command that is received.
The message parameter contains a string describing the command.
When a command is received a Grijper::command is created to give the controller command to the actuator.
This command contains a 'cmd' string describing the command and (if applicable) a 'force' float containing the target force.

\param msg The message containing the command string

*/
void gripperCommand(const std_msgs::String::ConstPtr& msg){
	string cmd = msg->data;
	Grijper::command ctl;
	if(cmd.compare("open") == 0){
		ROS_INFO("Opening gripper");
		force_open = true;
		ctl.cmd = "open";
		ctl.force = force;
		gripper_open = true;
		control.publish(ctl);
	}else if(cmd.compare("relax") == 0){
		ROS_INFO("Relaxing gripper");
		force_open = false;
		ctl.cmd = "relax";
		ctl.force = 0;
		control.publish(ctl);
	}else if(cmd.compare("close") == 0){
		ROS_INFO("Closing gripper");
		force_open = false;
		ctl.cmd = "close";
		ctl.force = force;
		gripper_open = false;
		control.publish(ctl);
	}else{
		ROS_INFO("Received invalid command: %s\n", cmd.c_str());
	}
}

/*! \brief Small evaluation method to see if the gripper should be closed.

Compares the distance value to a threshold value to see if the gripper should be closed.

\param distance The distance to be evaluated.
\return Boolean whether the gripper should be closed.

*/
bool close_gripper(float distance){
	return distance < 7 && distance != 0; //TODO dit nog bewerken afhankelijk van de sensor.
}

/*! \brief Small evaluation method to see if the gripper should be opened.

Compares the distance value to a threshold value to see if the gripper should be opened.

\param distance The distance to be evaluated.
\return Boolean whether the gripper should be opened.

*/
bool open_gripper(float distance){
	return distance > 10;
}

/*! \brief Controll method to shutdown this ROS node when the command is given.

This method will shutdown this node when the command is given by the console.
\param b The message containing the command. Nothing is done with the command since we know what it will be when this message is received.
*/
void shutdown(const std_msgs::Bool::ConstPtr& b){
	ros::shutdown();
}