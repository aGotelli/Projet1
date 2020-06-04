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

#include "simulation/ground_generation.h" // WRONG HEADER FILE NAME
#include "simulation/utility.h"
#include "simulation/sensor.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>



//	Position of the robot
geometry_msgs::Pose robotPosture;

void RobotPostureReceived(const geometry_msgs::PoseStamped::ConstPtr& _robotPosture)
{
	robotPosture = _robotPosture->pose;
}

//	Line marker
visualization_msgs::Marker tilesHLine, tilesVLine;


int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "ground_generation");

    ros::NodeHandle nh_glob;


    //  Global world parameters
  	double xSpacing, ySpacing, lineThickness;					// [m]
  	nh_glob.param("x_spacing", xSpacing, 0.2) ;
  	nh_glob.param("y_spacing", ySpacing, 0.2) ;
  	nh_glob.param("line_thickness", lineThickness, 0.005) ;

  	//  Ground generator parameters
  	int xLines, yLines;				// number of lines
  	nh_glob.param("x_lines", xLines, 4) ;
  	nh_glob.param("y_lines", yLines, 4) ;


    // Declare your node's subscriptions and service clients
	ros::Subscriber RecivedPosture = nh_glob.subscribe<geometry_msgs::PoseStamped>("RobotPosture", 1, RobotPostureReceived);

    // Declare you publishers and service servers
	ros::Publisher TilesDisplay = nh_glob.advertise<visualization_msgs::Marker>("/visualization_marker", 1);


	TileGeneration(tilesHLine, tilesVLine);

    ros::Rate rate(150);
    while (ros::ok()){
        ros::spinOnce();


        UpdateTiles(world.oTm(), tilesHLine, tilesVLine);


        TilesDisplay.publish( tilesHLine ) ;
				TilesDisplay.publish( tilesVLine ) ;

        rate.sleep();
    }
}
