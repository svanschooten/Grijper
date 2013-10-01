#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <string>
#include <phidget_ik/phidget_ik.h>
#include <Grijper/command.h>

using namespace std;

int sensor_id = 4;
int freq = 10;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "phidget_reader");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<std_msgs::Int32>("phidget_value", 1000);
	ros::Rate loop_rate(freq);
	PhidgetIK phidget_ik;
  
	phidget_ik.init(-1);
	phidget_ik.waitForAttachment(1000);

	if (phidget_ik.getLastError()){
		std::cerr << "Error initializing PhidgetInterfaceKit: " << phidget_ik.getErrorDescription(phidget_ik.getLastError()) << std::endl;
		return 1;
	}

	while(ros::ok()){
		int data = phidget_ik.getSensorValue(sensor_id);
		std_msgs::Int32 sensor_val;

		if(data < 100)
			ROS_INFO("Ignoring sensor value of %i", data);
		else{
			sensor_val.data = data;
			ROS_INFO("Reading sensor %i, value: %i",sensor_id, data);

			publisher.publish(sensor_val);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
