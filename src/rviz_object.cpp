#include <ros/ros.h>
#include <string>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
//#include <rvix> en dan nog wat dingen

void rvizUpdater(const std_msgs::Int32::ConstPtr&);
// declare variables 
int dist_int;

int main(int argc, char **argv){
	
	double dist;
	bool added = false;
	double diameter = 0.06, height=0.1;;

	ros::init(argc, argv, "rviz_object");
	ros::NodeHandle n;
	ros::Subscriber subscriber = n.subscribe("phidget_value", 1000, rvizUpdater);
	ros::Publisher cylinder_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Rate loop_rate(10);
	ros::spinOnce();
	

	while (ros::ok())
  	{
  		// message
		visualization_msgs::Marker marker;
		// marker frame
		marker.header.frame_id = "base";
		// timestamp
		marker.header.stamp = ros::Time::now();
		// marker name
		marker.ns="shape_cylinder";
		// marker id
		marker.id=0;
		// marker shape
		marker.type = visualization_msgs::Marker::CYLINDER;
		// add marker

    	if(dist_int)
    	{
    		// distance 2 double
			dist = dist_int/100;

			// add marker if there is no marker
    		if(added==false){
				marker.action = visualization_msgs::Marker::ADD;
				added = true;
			}
    		
			//marker position
			marker.pose.position.x = 0;
			marker.pose.position.y = -dist;
			marker.pose.position.z = height/2-0.02;
			// marker orientation
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;

			// scale shape
			marker.scale.x = diameter;
			marker.scale.y = diameter;
			marker.scale.z = height;

			// color
			marker.color.r = 0.0f;
			marker.color.g = 1.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0;

			// lifetime
			marker.lifetime = ros::Duration();

   			cylinder_pub.publish(marker);
    	}else if(added == true){
    		// if there is no object in the range of the sensor delete the marker
    		marker.action = visualization_msgs::Marker::DELETE;
			added = false;
			cylinder_pub.publish(marker);
    	}
    	ros::spinOnce();
    	loop_rate.sleep();
  	}
	
	return 0;
}

void rvizUpdater(const std_msgs::Int32::ConstPtr& msg){
	dist_int = msg->data;
	ROS_INFO("rziv distance: %i", msg->data);
}
