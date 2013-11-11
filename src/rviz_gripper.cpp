#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
//#include <rvix> en dan nog wat dingen

using namespace std;

void rvizUpdater(const std_msgs::Float32::ConstPtr&);
void shutdown(const std_msgs::Bool::ConstPtr&);
float opens; /*!< The state of the gripper 0 when gripper closes, 1 when gripper opens, 0.5 in relax mode. */
const int freq = 250; /*!< The frequency at which gripper in rviz is updated. */

/*! \brief Main method reads if the state of the gripper is changing and simulates this in RVIZ.

This node listens to the gripper_state published by gripper_actuator.

The motor_state is only changed if the gripper opens or closes and the motor state is between 1 and 0
The new motor_state is converted to angles for the both joints.
The new angles will be published in the jstates topic
 
\param motor_state variable has a value between 1 and 0. 1 means completly open 0 means completly closed.
\param bmdll_max angle that the joint between middle and base can make.
\paramm lltp_max angle that the joint between middle and finger tip can make.
\param bmdll_start start angle of the joint between middle and base
\param mdlltp_start start angle of the joint between middle and tip
\param bm_angle current angle between base and middle
\param mt_angle current angle between middle and tip
\param step_size the change of motor_state per loop
\param cf corection facter the factor the raid at which the tip moves faster then the middle part
\param angle_step the inverse of the total angle that can be made by the two joints

*/

int main(int argc, char **argv){
	
	ros::init(argc, argv, "rviz_gripper");
	ros::NodeHandle n;
	ros::Subscriber subscriber = n.subscribe("gripper_state", 1000, rvizUpdater);
	ros::Subscriber sd = n.subscribe("shutdown", 1, shutdown);
	ros::spinOnce();
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("jstates", 1);
	ros::Rate loop_rate(freq);
	ros::spinOnce();

	const double bmdll_max = 0.85;
	const double mdlltp_max = 0.80;
	const double bmdll_start = -0.96;
	const double mdlltp_start = -0.3;
	double bm_angle, mt_angle, step_size = 0.004, cf = 1.5, motor_state = 0;
	double angle_step = 1/(bmdll_max + mdlltp_max*cf);

	sensor_msgs::JointState joint_state;

	while (ros::ok())
	{	
		if((opens == 1 && motor_state < (1-step_size)) || (opens == 0 && motor_state > step_size)){
			if(opens == 0){
					motor_state = motor_state - step_size;
				
			}else{
					motor_state = motor_state + step_size;
				
			}
			if((motor_state/angle_step) < bmdll_max){
				mt_angle = mdlltp_start;
				bm_angle = motor_state/angle_step+bmdll_start;
			}else{
				bm_angle = bmdll_start+bmdll_max;
				mt_angle = (motor_state/angle_step)-bmdll_max+mdlltp_start;
			}
			joint_state.header.stamp = ros::Time::now();
			joint_state.name.resize(2);
			joint_state.position.resize(2);

			joint_state.name[0]="rght_base2mdll";
			joint_state.position[0] = bm_angle;

			joint_state.name[1]="rght_mdll2tip";
			joint_state.position[1] = mt_angle;
			joint_pub.publish(joint_state);
		}
		ros::spinOnce();
    	loop_rate.sleep();
		
	}

	return 0;
}

/*! \brief update method the opens variable will be updated

\param msg contains the value 0,1,0.5 depending if the gripper is openening, closing or relaxing.
*/

void rvizUpdater(const std_msgs::Float32::ConstPtr& msg){
	opens = msg->data;
	ROS_INFO("open or close: %f", msg->data);
}

/*! \brief Controll method to shutdown this ROS node when the command is given.

This method will shutdown this node when the command is given by the console.
\param b The message containing the command. Nothing is done with the command since we know what it will be when this message is received.
*/

void shutdown(const std_msgs::Bool::ConstPtr& b){
	ros::shutdown();
}