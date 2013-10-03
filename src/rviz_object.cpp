#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
//#include <rvix> en dan nog wat dingen


void rvizUpdater(const std_msgs::Float32::ConstPtr&);
void shutdown(const std_msgs::Bool::ConstPtr&);
ros::Publisher cylinder_pub; /*!< cylinder publisher. */

//const int freq = 250; /*!< The frequency at which the subscriber reads the . */
bool added = false; /*!< Cylinder added value */

/*! \brief Main method of the rviz_object node. This node evaluates the distance published by phidget_reader.

If there is an object in sight of the sensor this nod publishes a marker at the distance measured by the sensor in rviz. 
If there is no object in the sight of the sensor the cylinder will be deleted.
This node also listens to the shutdown command given by the controller.
*/


int main(int argc, char **argv){

	ros::init(argc, argv, "rviz_object");
	ros::NodeHandle n;
	ros::Subscriber subscriber = n.subscribe("phidget_value", 1000, rvizUpdater);
	ros::Subscriber sd = n.subscribe("shutdown", 1, shutdown);
	cylinder_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::spin();
	
	
	return 0;
}

/*! \brief Method for publishing a cylinder in rviz

This Method creates a marker message to create a cylinder in rviz.
The frame where the marker in will be plotted in rviz is the base frame of the gripper.
The marker will be deleted if there is no object in the range of the sensor.
The type of the marker is a cylinder. Marker will be placed on the y-axis on the measured distance from the origin
The size of the marker is determined with the diameter and height variable.
The color of the marker is green.
*/

void rvizUpdater(const std_msgs::Float32::ConstPtr& msg){
	const double diameter = 0.06, height=0.1;;
	float distance;
	distance = msg->data;
	ROS_INFO("rziv distance: %f", distance);
	
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base";
	marker.header.stamp = ros::Time::now();
	marker.ns="shape_cylinder";
	marker.id=0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	if(distance == 0){
		if(added == true){
			marker.action = visualization_msgs::Marker::DELETE;
			added = false;
			cylinder_pub.publish(marker);
		}
	}else{
    	if(added==false){
			marker.action = visualization_msgs::Marker::ADD;
			added = true;
		}
    
		marker.pose.position.x = 0;
		marker.pose.position.y = -distance/100;
		marker.pose.position.z = height/2-0.02;
		
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		marker.scale.x = diameter;
		marker.scale.y = diameter;
		marker.scale.z = height;
		
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		cylinder_pub.publish(marker);
	}
}

/*! \brief Controll method to shutdown this ROS node when the command is given.

This method will shutdown this node when the command is given by the console.
\param b The message containing the command. Nothing is done with the command since we know what it will be when this message is received.
*/

void shutdown(const std_msgs::Bool::ConstPtr& b){
	ros::shutdown();
}
