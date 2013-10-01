#include <stdio.h>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Grijper/command.h>
#include <ros/ros.h>

using namespace std;

float force = 0.5;
bool exit_cmd = false;
ros::Publisher force_ad;

void init();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_console");
  ros::NodeHandle n;
  force_ad = n.advertise<std_msgs::Float32>("gripper_force", 1000);
  ros::Publisher command_ad = n.advertise<std_msgs::String>("command", 1000);
  ros::Publisher shutdown = n.advertise<std_msgs::Bool>("shutdown", 1);
  init();
 
  while(exit_cmd == false){
    string cmd;
    printf(">> ");
    cin >> cmd;
    Grijper::command command;
    command.force = force;
    if(cmd.compare("open") == 0){
      std_msgs::String msg;
      msg.data = "open";
      command_ad.publish(msg);
    }else if(cmd.compare("close") == 0){
      std_msgs::String msg;
      msg.data = "close";
      command_ad.publish(msg);
    }else if(cmd.compare("relax") == 0){
      std_msgs::String msg;
      msg.data = "relax";
      command_ad.publish(msg);
    }else if(cmd.compare("get_force") == 0){
      printf("Force is: %f\n", force);
    }else if(cmd.compare("force") == 0){
      printf("Set force to: ");
      cin >> force;
      std_msgs::Float32 msg;
      msg.data = force;
      force_ad.publish(msg);
      printf("Force set to: %f\n", force);
    }else if(cmd.compare("exit") == 0){
      exit_cmd = true;
      std_msgs::Bool msg;
      msg.data = true;
      shutdown.publish(msg);
    }else{
      printf("Please use the commands 'open', 'close', 'relax', 'force', 'get_force' or 'exit'.\n");
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
