#include <ros/ros.h>
#include <Grijper/calc_current.h>

bool calc(Grijper::calc_current::Request&, Grijper::calc_current::Response&)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calc_current");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("calc_current", calc);
  ROS_INFO("Ready to calculate needed current.");
  ros::spin();

  return 0;
}

bool calc(Grijper::calc_current::Request  &req, Grijper::calc_current::Response &res){
  res.current = req.force; //TODO hier moet nog de berekening gedaan worden.
  ROS_INFO("request: %f nM", req.force);
  ROS_INFO("sending back response: %f A", res.current);
  return true;
}