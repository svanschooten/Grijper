#include <stdio.h>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Grijper/command.h>
#include <ros/ros.h>

using namespace std;

float force = 0.35; /*!< Force variable standard set to 0.35. */
bool exit_cmd = false; /*!< Variable indicating whether the shutdown command has been given. */
ros::Publisher force_ad; /*!< Force message publisher. */

void init();

/*! \brief The main method of the console. The console listens to input an passes the commands on to the controller

The console listens to user input to generate a command.
These commands can be 'open', 'close', 'relax', 'force < target force >', 'get_force' and 'exit'
The 'open' command manually opens the gripper.
The 'close' command manually closes the gripper.
The 'relax' command relaxes the gripper (e.g. set the target force to 0). This can only be done manually.
The 'force < target force >' command sets the target force to the '< target force >' parameter.
The 'get_force' command returns the target force as has been set.
The 'exit' command passes on the shutdown command to all listening nodes before shutting down itself. This should be used to control all nodes.

If an invalid command is given it will return "Please use the commands 'open', 'close', 'relax', 'force', 'get_force' or 'exit'.".'

*/
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
      printf("Force is: %f nM\n", force);
    }else if(cmd.compare("force") == 0){
      printf("Set force to: ");
      cin >> force;
      std_msgs::Float32 msg;
      msg.data = force;
      force_ad.publish(msg);
      printf("Force set to: %f nM\n", force);
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

/*! \brief Initialiser for the system to ensure the default forces are identical throughout the system

Initialises the system, or in this case, passes on the given default target force for consistency ensurance.
*/
void init(){
  std_msgs::Float32 msg;
  msg.data = force;
  force_ad.publish(msg);
}
