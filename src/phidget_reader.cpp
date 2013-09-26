#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sstream>
#include <phidget_ik/phidget_ik.h>

using namespace std;

int sensor_id = 0;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "phidget_reader");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<std_msgs::Int32>("phidget_value", 1000);
	ros::Rate loop_rate(10);

	int count = 0; //TODO mag zometeen weg
	while(ros::ok()){
		int data = count;//PhidgetIK::getSensorValue(sensor_id)
		std_msgs::Int32 sensor_val;
		sensor_val.data = data;
		ROS_INFO("Reading sensor %i, value: %i",sensor_id, data);

		publisher.publish(sensor_val);

		ros::spinOnce();
		loop_rate.sleep();

		++count;
	}

	return 0;
}