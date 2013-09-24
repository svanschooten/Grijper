#include <stdio.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

float force = 0.5;
bool exit = false;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_console");
  ros::NodeHandle n;
  force_ad = n.advertise<std_msgs::Float32>("gripper_force", 1000);
  command_ad = n.advertise<std_msgs::String>("gripper_cmd", 1000);
  init();
 
  while(!exit){
    string cmd;
    float val;
    cin >> cmd >> val;
    Grijper::gripper_cmd command;
    command.force = force;
    switch(cmd){
      case: "open":
        std_msgs::String msg;
        msg.data = "open";
        command_ad.publish(msg);
        break;
      case "close":
        std_msgs::String msg;
        msg.data = "close";
        command_ad.publish(msg);
        break;
      case "force":
        if(val != NULL){
          force = val;
          std_msgs::Float32 msg;
          msg.data = force;
          force_ad.publish(msg);
          printf("Force set to: %f\n", force);
        } else {
          printf("Force is: %f\n", force);
        }
        break;
      case "exit":
        exit = true;
        break;
      default:
        printf("Please use the commands 'open', 'close', 'force <val>' or 'exit'.\n");
        break;
    }
  }
  
  ros::shutdown();
  
  return 0;
} 

void init(){
  std_msgs::Float32 msg;
  msg.data = force;
  force_ad.publish(msg);
}