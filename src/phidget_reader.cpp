#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float32.h>
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
float sensorToDistance(int);

const int sensor_id = 4; /*!< The sensor ID. */
const int freq = 250; /*!< The frequency at which the sensor should be read and the value passed on. */
const int buffer_size = 25; /*!< Value buffer size for signal smoothing. */
int buffer[buffer_size];  /*!< The buffer used for smoothing. */


/*! \brief Main method reading the phidget at a specified frequency and smoothing the signal before passing it on.

Here the phidget is read at a fixed interval, given by freq. These values are stored in the buffer and sliding mean smoothing is applied.
This ensures a less spikey signal for the controller and impulses are filtered out.
This also means that the system has a longer response time, but by increasing the frequency this is nullified.
The buffer acts as a queue, shifting the values every time before calculating a mean value to pass on to the controller.
\sa shiftBuffer(int), mean()
*/
int main(int argc, char **argv){
	
	ros::init(argc, argv, "phidget_reader");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<std_msgs::Float32>("phidget_value", 1000);
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
		std_msgs::Float32 sensor_val;
		shiftBuffer(data);
		data = mean();

		if(data < 100 || data > 700){
			//printf("Ignoring sensor value of %i\n", data);
		}else{
			float distance = sensorToDistance(data);
			sensor_val.data = distance;
			printf("Reading sensor %i, value: %i, distance:%f\n",sensor_id, data, distance);
			//printbuffer();

			publisher.publish(sensor_val);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

/*! \brief Method for shifting the buffer and inserting the newest sensor value.

This method shifts all the data elements in the buffer to make place for the new sensor data.
The buffer is initialised as an array filled with '1', so the first buffer_size values will be invalid.
\param data The new sensor value to be shifted into the buffer.
\sa mean()
*/
void shiftBuffer(int data){
	for(int i = 0; i < buffer_size - 1; i++){
		buffer[i] = buffer[i+1];
	}
	buffer[buffer_size - 1] = data;
}

/*! \brief Method for calulating the mean of the buffer to pass on to the controller

This method calculated the mean of the values in the buffer.
\return The calculated mean.
\sa shiftBuffer(int)
*/
int mean(){
	int size = 0;
	for(int i = 0; i < buffer_size; i
		++){
		size = size + buffer[i];
	}
	return(ceil(size/buffer_size));
}

/*! \brief A small control method to print the buffer.

Prints the buffer for each iteration so the calculated mean value can be evaluated.
\sa mean()
*/
void printbuffer(){
	cout << " - [";
	for(int i = 0; i < buffer_size; i++){
		printf("%i,", buffer[i]);
	}
	cout << "]\n";
}

/*! \brief simple calculation method to extract a distance from the sensor value.

This method converts the sersorvalue to a distance.

\param sensorValue The value of the sensor.
\return The calculated distance.

*/
float sensorToDistance(int sensorValue){

	return 2076.0f / (sensorValue - 11.0f);

}

/*! \brief Controll method to shutdown this ROS node when the command is given.

This method will shutdown this node when the command is given by the console.
\param b The message containing the command. Nothing is done with the command since we know what it will be when this message is received.
*/
void shutdown(const std_msgs::Bool::ConstPtr& b){
	ros::shutdown();
}