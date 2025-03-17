#include <iostream>
#include <thread>
#include <termios.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(fileno(stdin), TCSANOW, &newt);
  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub;

public:
  double vel_x, ang_z;
  char cmd[50];
  
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;

    vel_x = 0, ang_z = 0;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {
    std::cout << "Type a command and then press enter.  "
      "Use 'w' to move forward, 'a' to turn left, "
      "'d' to turn right, 's' to stop, " 
	  "'x' to move back, '.' to exit.\n";

    
    while(nh_.ok()){

//      std::cin.getline(cmd, 50);
	  cmd[0] = getch();
      if(cmd[0]!='w' && cmd[0]!='a' && cmd[0]!='d' && cmd[0]!='s' && cmd[0]!='x' && cmd[0]!='.')
      {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      //move forward
      if(cmd[0]=='w'){
        if(vel_x < 0.22)vel_x+=0.044;
      }
      //turn left (yaw) and drive forward at the same time
      else if(cmd[0]=='a'){
        if(ang_z < 2.84)ang_z+=0.568;
      }
      //turn right (yaw) and drive forward at the same time
      else if(cmd[0]=='d'){
        if(ang_z > -2.84)ang_z-=0.568;
      }
      //stop
      else if(cmd[0]=='s'){
        vel_x = 0;
        ang_z = 0;
	  }
      //move back
      else if(cmd[0]=='x'){
        if(vel_x > -0.22)vel_x-=0.044;
	  }
      //quit
      else if(cmd[0]=='.'){
        vel_x = 0;
        ang_z = 0;
        break;
      }
//      std::cout << "linear.x :" << vel_x << "\n";
//      std::cout << "angular.z :" << ang_z << "\n";
	  printf("linear.x : %.3lf\n" , vel_x);
	  printf("angular.z : %.3lf\n\n" , ang_z);

    }
    return true;
  }

  bool send_msg_thread()
  {
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;

    ros::Rate loop_rate(3);

    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    while(1)
    {
      //publish the assembled command
      base_cmd.linear.x = vel_x;
      base_cmd.angular.z = ang_z;
      cmd_vel_pub.publish(base_cmd);
      if(cmd[0]=='.')break;
      
      loop_rate.sleep();
    }
    return true;
  }

  void robot_driver_start()
  {
    std::thread t1(&RobotDriver::driveKeyboard, this);
    std::thread t2(&RobotDriver::send_msg_thread, this);
    t1.join();
    t2.join();
  }
};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  driver.robot_driver_start();
}
