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
          and will output a feasible twist for the robot. As it is an adapter
          component, it will be as short as possible in terms of code length.

 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for movement keys
std::map<char, std::vector<double>> motionQWERTY
{
  //  Configuration for the QWERTY
  {'w', {1, 0}},
  {'s', {-1, 0}},
  {'a', {0, 1}},
  {'d', {0, -1}},
  {'x', {0, 0}},

  {'q', {1, 1}},
  {'e', {1, -1}},
  {'z', {-1, 1}},
  {'c', {-1, -1}}

};

// Map for movement keys
std::map<char, std::vector<double>> motionAZERTY
{
  //  Configuration for the AZERTY
  {'z', {1, 0}},
  {'s', {-1, 0}},
  {'q', {0, 1}},
  {'d', {0, -1}},
  {'x', {0, 0}},

  {'a', {1, 1}},
  {'e', {1, -1}},
  {'w', {-1, 1}},
  {'c', {-1, -1}}

};



// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------------------------------
 Moving around:      |    Moving around (AZERTY)
 q    w    e         |      a    z    e
 a    s    d         |      q    s    d
 z         c         |      w         c

               Stop the robot:
                     x

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

  ros::NodeHandle nh_glob, nh_loc("~");

  // Declare your publishers and service servers
  ros::Publisher robotControl = nh_glob.advertise<geometry_msgs::Twist>("/TwistToRobot", 1);

  //  Show how to use the keyboard
  ROS_INFO( "%s", msg );

  //  The control message
  geometry_msgs::Twist twistToRobot;

  double v = 1.0f; // Linear velocity (m/s)
  double omega = 1.0f; // Angular velocity (rad/s)

  double scale_v = 1.0f;
  double scale_omega = 1.0f;

  bool QWERTY;
  nh_loc.param<bool>("QWERTY", QWERTY, true);

  std::map<char, std::vector<double>> motion;

  if(QWERTY)
    motion = motionQWERTY;
  else
    motion = motionAZERTY;

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


    // Otherwise, quit the simulation ( CTRL C )
    else if ( key == '\x03' ) { break; }


    // Update the Twist message
    twistToRobot.linear.x = v;
    twistToRobot.angular.z = omega;

    // Publish it and resolve any remaining callbacks
    robotControl.publish( twistToRobot );


    rate.sleep();

  }
}
