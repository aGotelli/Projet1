/**
 * \file compute twist
 * \brief Adapter component for translate the joint button state into a command
 * \author Bianca Lento & Andrea Gotelli
 * \version 0.1
 * \date 28/05/2020
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    ° /joy
 *
 * Publishes to: <BR>
 *    ° /TwistToRobot
 *
 * Description
          This node will take the status of the controller, i.e. the joystick,
          and will output a feasible twist for the robot. As it is an adapeter
          component, it will be as short as possible in terms of code length.

          In this first version, the Andrea's joystic is the only one implemented.
 *
 */


//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic type you use.
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include "simulation/robot_2_0_propose.h"


geometry_msgs::Twist twistToRobot;
bool joystickIsReceived;

// Callback to update the joystick state
void JoystickStateUpdate(const sensor_msgs::Joy::ConstPtr& joystick)
{

/* The joy topic related to Andrea's joystick:

header:
  seq: 2
  stamp:
    secs: 1590675493
    nsecs: 159840453
  frame_id: ''
axes: [-0.0, 0.0]
      [ right/left, up/down    ]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
buttons: [1, 2, 3, 4, L1, R1, L2, R2, SELECT, START]

*/
  double Kv = 1 ;

  if(joystick->buttons[0])
    Kv = 0.25*joystick->buttons[0] ;

  if(joystick->buttons[1])
    Kv = 0.5*joystick->buttons[1] ;

  if(joystick->buttons[2])
    Kv = 1*joystick->buttons[2] ;

  if(joystick->buttons[3])
    Kv = 1.5*joystick->buttons[3] ;

  // Obtaining data from joystick
  double v = joystick->axes[1] ;
  double omega = joystick->axes[0] ;

  double wl = 0.25*joystick->buttons[4] + 0.75*joystick->buttons[6] ;
  double wr = 0.25*joystick->buttons[5] + 0.75*joystick->buttons[7] ;

  // Computation of velocities
  twistToRobot.linear.x = Kv*v;
  twistToRobot.angular.z = Kv*(omega + (wl - wr));

  if( !joystickIsReceived)
    joystickIsReceived = true;
}




int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "compute_twist");

    ros::NodeHandle nh_glob;

    // Read the node parameters if any
    // Not for version 1

    // Declare your node's subscriptions and service clients
    ros::Subscriber joystickState = nh_glob.subscribe<sensor_msgs::Joy>("/joy", 1, JoystickStateUpdate) ;

    // Declare you publishers and service servers
    ros::Publisher robotControl = nh_glob.advertise<geometry_msgs::Twist>("/TwistToRobot", 1) ;


    ros::Rate rate(150);
    while (ros::ok()){
        ros::spinOnce();

        // Not really much to do here..
        if( !joystickIsReceived )
          continue;

        robotControl.publish( twistToRobot ) ;

        rate.sleep();
    }
}
