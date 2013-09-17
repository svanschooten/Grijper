#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>

int main(int argc, char **argv){
	
	ros::init(argc, argv, "phidget_reader");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<std_msgs::Int32>("phidget_values", 1000);
	ros::Rate loop_rate(10);

	int count = 0; //TODO mag zometeen weg
	while(ros::ok()){
		std_msgs::Int32 sensor_val;
		sensor_val.data = count; //TODO hier moet sensor data uitgelezen worden.
		ROS_INFO("%i", sensor_val.data);

		publisher.publish(sensor_val);

		ros::spinOnce();
		loop_rate.sleep();

		++count;
	}

	return 0;
}