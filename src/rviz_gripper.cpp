#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
//#include <rvix> en dan nog wat dingen

using namespace std;

void rvizUpdater(const std_msgs::Float32::ConstPtr&);
void shutdown(const std_msgs::Bool::ConstPtr&);
float opens;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "rviz_gripper");
	ros::NodeHandle n;
	ros::Subscriber subscriber = n.subscribe("gripper_state", 1000, rvizUpdater);
	ros::Subscriber sd = n.subscribe("shutdown", 1, shutdown);
	ros::spinOnce();
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("jstates", 1);
	ros::Rate loop_rate(100);
	ros::spinOnce();

	// declare const
	const double bmdll_max = 0.85;
	const double mdlltp_max = 0.80;
	// declare variables
	const double bmdll_start = -0.96;
	const double mdlltp_start = -0.3;
	double bm_angle, mt_angle, step_size = 0.01, cf = 1, motor_stand = 0;

	// calculate step_size and total angle
	double angle_step = 1/(bmdll_max + mdlltp_max*cf);

	// message declarations
	sensor_msgs::JointState joint_state;

	while (ros::ok())
	{

		if((opens == 0 || motor_stand == 1)&&(motor_stand+step_size<1 && motor_stand-step_size>0)){
			if(opens == 0){
				//gripper closes
				motor_stand = motor_stand - step_size;
			}else{
				// gripper opens
				motor_stand = motor_stand + step_size;
			}
			if(motor_stand*angle_step < bmdll_max){
				mt_angle = mdlltp_start;
				bm_angle = motor_stand*angle_step;
			}else{
				bm_angle = bmdll_start+bmdll_max;
				mt_angle = (motor_stand*angle_step)-bmdll_max;
			}
			joint_state.header.stamp = ros::Time::now();
			joint_state.name.resize(2);
			joint_state.position.resize(2);

			joint_state.name[0]="rght_base2mdll";
			joint_state.position[0] = bm_angle;

			joint_state.name[1]="rght_mdll2tip";
			joint_state.position[1] = mt_angle;

			// send joint state
			joint_pub.publish(joint_state);
		}

		//ROS_INFO("mt_angle: [%f]", mt_angle);

		//std::cout<<bmdll;
		ros::spinOnce();
    	loop_rate.sleep();
		
	}

	return 0;
}

void rvizUpdater(const std_msgs::Float32::ConstPtr& msg){
	opens = msg->data;
	ROS_INFO("open or close: %f", msg->data);
}

void shutdown(const std_msgs::Bool::ConstPtr& b){
	ros::shutdown();
}