#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
//#include <rvix> en dan nog wat dingen

using namespace std;

void rvizUpdater(const std_msgs::Float32::ConstPtr&);
void shutdown(const std_msgs::Bool::ConstPtr&);
float motor_stand;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "rviz_gripper");
	ros::NodeHandle n;
	ros::Subscriber subscriber = n.subscribe("gripper_state", 1000, rvizUpdater);
	ros::Subscriber sd = n.subscribe("shutdown", 1, shutdown);
	ros::spinOnce();
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("jstates", 1);
	ros::Rate loop_rate(10);
	ros::spinOnce();

	// declare const
	const double bmdll_max = 0.85;
	const double mdlltp_max = 0.80;
	// declare variables
	double bmdll= 0;
	double mdlltp = 0;
	double bm_angle = -0.96, mt_angle = -0.3, angle_tot, step_size, cf = 1;

	// calculate step_size and total angle
	angle_tot = bmdll_max + mdlltp_max*cf;
	step_size = 1/angle_tot;

	// message declarations
	sensor_msgs::JointState joint_state;

	while (ros::ok())
	{

		if(motor_stand <= bmdll_max*step_size)
		{
			bmdll = motor_stand/step_size;
			bm_angle = -0.96+bmdll;
		}else{
			mdlltp = (motor_stand/step_size - bmdll_max)/cf;
			mt_angle = -0.3+mdlltp;
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

		//ROS_INFO("mt_angle: [%f]", mt_angle);

		//std::cout<<bmdll;
		ros::spinOnce();
    	loop_rate.sleep();
		
	}

	return 0;
}

void rvizUpdater(const std_msgs::Float32::ConstPtr& msg){
	motor_stand = msg->data;
	ROS_INFO("rziv distance: %f", msg->data);
}

void shutdown(const std_msgs::Bool::ConstPtr& b){
	ros::shutdown();
}