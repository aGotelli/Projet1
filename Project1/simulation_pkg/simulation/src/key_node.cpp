#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for movement keys
std::map<char, std::vector<double>> motion
{
  // {'i', {1, 0, 0}},
  // {'o', {1, 0, -1}},
  // {'j', {0, 0, 1}},
  // {'l', {0, 0, -1}},
  // {'u', {1, 0, 1}},
  // {',', {-1, 0, 0}},
  // {'.', {-1, 0, 1}},
  // {'m', {-1, 0, -1}},
  // {'k', {0, 0, 0}},

  {'w', {1, 0, 0}},
  {'s', {-1, 0, 0}},
  {'a', {0, 0, 1}},
  {'d', {0, 0, -1}},


};

// Map for speed keys
std::map<char, std::vector<double>> speed
 {
//   {'q', {1.1, 1.1}},
//   {'z', {0.9, 0.9}},
//   {'w', {1.1, 1}},
//   {'x', {0.9, 1}},
//   {'e', {1, 1.1}},
//   {'c', {1, 0.9}}
};

// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .


q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
)";

// Init variables
// double v; // Linear velocity (m/s)
// double omega; // Angular velocity (rad/s)
// double x, y, theta; // Forward/backward/neutral direction vars
char key(' ');
geometry_msgs::Twist twistToRobot;



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


  ROS_INFO( "%s", msg );

  double v = 1; // Linear velocity (m/s)
  double omega = 1; // Angular velocity (rad/s)
  double x, y, theta; // Forward/backward/neutral direction vars

  ros::Rate rate(150);
  while(ros::ok()){
    ros::spinOnce();

    // Get the pressed key
    key = getch();

    ROS_INFO_STREAM(" typed : " << key ) ;

    // If the key corresponds to a key in moveBindings
    if ( motion.count(key) == 1 )
    {
      // Grab the direction data
      // x = motion[key][0];
      // y = motion[key][1];
      // theta = motion[key][2];

      v = motion[key][0];
      omega = motion[key][2];

    }

    // Otherwise if it corresponds to a key in speedBindings
    else if ( speed.count(key) == 1 )
    {
      // Grab the speed data
      v = v * speed[key][0];
      omega = omega * speed[key][1];

    }

    // Otherwise, stop the robot (CTRL C)
    else if ( key == '\x03' )
    {
      break;
    }



    // Update the Twist message
    twistToRobot.linear.x = v;
    twistToRobot.angular.z = omega;

    // Publish it and resolve any remaining callbacks
    robotControl.publish( twistToRobot );
    v = 0;
    omega = 0;

    rate.sleep();

  }
}
