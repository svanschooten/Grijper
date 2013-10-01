#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <string>
#include <phidget_ik/phidget_ik.h>
#include <Grijper/command.h>

using namespace std;

void shiftBuffer(int);
int mean();
void printbuffer();
void shutdown(const std_msgs::Bool::ConstPtr&);

const int sensor_id = 4;
const int freq = 250;
const int buffer_size = 25;
int buffer[buffer_size];

int main(int argc, char **argv){
	
	ros::init(argc, argv, "phidget_reader");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<std_msgs::Int32>("phidget_value", 1000);
	ros::Subscriber sd = n.subscribe("shutdown", 1, shutdown);
	ros::Rate loop_rate(freq);
	PhidgetIK phidget_ik;
  
	phidget_ik.init(-1);
	phidget_ik.waitForAttachment(1000);

	if (phidget_ik.getLastError()){
		std::cerr << "Error initializing PhidgetInterfaceKit: " << phidget_ik.getErrorDescription(phidget_ik.getLastError()) << std::endl;
		return 1;
	}

	for(int i = 0; i < buffer_size; i++)
	{
		buffer[i] = 1;
	}

	while(ros::ok()){
		int data = phidget_ik.getSensorValue(sensor_id);
		std_msgs::Int32 sensor_val;
		shiftBuffer(data);
		data = mean();

		if(data < 100 || data > 700){
			//printf("Ignoring sensor value of %i\n", data);
		}else{
			sensor_val.data = data;
			printf("Reading sensor %i, value: %i\n",sensor_id, data);
			//printbuffer();

			publisher.publish(sensor_val);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void shiftBuffer(int data){
	for(int i = 0; i < buffer_size - 1; i++){
		buffer[i] = buffer[i+1];
	}
	buffer[buffer_size - 1] = data;
}

int mean(){
	int size = 0;
	for(int i = 0; i < buffer_size; i
		++){
		size = size + buffer[i];
	}
	return(ceil(size/buffer_size));
}

void printbuffer(){
	cout << " - [";
	for(int i = 0; i < buffer_size; i++){
		printf("%i,", buffer[i]);
	}
	cout << "]\n";
}

void shutdown(const std_msgs::Bool::ConstPtr& b){
	ros::shutdown();
}