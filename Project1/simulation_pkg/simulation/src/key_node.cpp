/**
 * \file  key node
 * \brief Adapter component for translate the keys state into a command
 * \author Bianca Lento & Andrea Gotelli
 * \version 0.1
 * \date 07/06/2020
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *
 *
 * Publishes to: <BR>
 *    ° /TwistToRobot
 *
 * Description
          This node will take the status of the controller, i.e. the keyboard,
          and will output a feasible twist for the robot. As it is an adapeter
          component, it will be as short as possible in terms of code length.

 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for movement keys
std::map<char, std::vector<double>> motion
{
  {'w', {1, 0}},
  {'s', {-1, 0}},
  {'a', {0, 1}},
  {'d', {0, -1}},

    {'x', {0, 0}}
};

// Map for speed keys
std::map<char, std::vector<double>> speed
 {
    {'j', {1.1, 1}},
    {'k', {0.9, 1}},
    {'n', {1, 1.1}},
    {'m', {1, 0.9}}
};

// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------------------------------
Moving around:
    w
a   s   d

Stop the robot:
    x

j/k : increase/decrease only linear speed by 10%
n/m : increase/decrease only angular speed by 10%

CTRL-C to quit
)";

//  The command key
char key(' ');

// For non-blocking keyboard inputs
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
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv)
{
  //ROS Initialization
  ros::init(argc, argv, "key_node");

  ros::NodeHandle nh_glob;

  // Declare you publishers and service servers
  ros::Publisher robotControl = nh_glob.advertise<geometry_msgs::Twist>("/TwistToRobot", 1);

  //  Show how to use the keyboard
  ROS_INFO( "%s", msg );

  //  The control message
  geometry_msgs::Twist twistToRobot;

  double v = 1.0f; // Linear velocity (m/s)
  double omega = 1.0f; // Angular velocity (rad/s)

  double scale_v = 1.0f;
  double scale_omega = 1.0f;

  ros::Rate rate(50);

  while(ros::ok()){
    ros::spinOnce();

    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in motion
    if ( motion.count(key) == 1 )
    {
      v = motion[key][0];
      omega = motion[key][1];

    }

    // Otherwise if it corresponds to a key in speed
    else if ( speed.count(key) == 1 )
    {
      // Grab the speed data
      scale_v *= speed[key][0];
      scale_omega *= speed[key][1];

    }

    //  Limit the scale factor
    if( scale_v > 3 ) {
      scale_v = 3;
    } else if ( scale_v < 0.001 ) {
      scale_v = 0;
    }

    if( scale_omega > 3) {
      scale_omega = 3;
    } else if ( scale_omega < 0.001 ) {
      scale_omega = 0;
    }

    // Otherwise, stop the robot (CTRL C)
    else if ( key == '\x03' ) { break; }



    // Update the Twist message
    twistToRobot.linear.x = v*scale_v;
    twistToRobot.angular.z = omega*scale_omega;

    // Publish it and resolve any remaining callbacks
    robotControl.publish( twistToRobot );


    rate.sleep();

  }
}
