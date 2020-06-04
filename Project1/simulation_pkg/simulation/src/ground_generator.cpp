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

#include "simulation/ground.h"






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

  ros::NodeHandle nh_glob, nh_loc("~");


  //  Global world parameters
	double xSpacing, ySpacing, lineThickness;					// [m]
	nh_loc.param("x_spacing", xSpacing, 1.0) ;
	nh_loc.param("y_spacing", ySpacing, 1.0) ;
	nh_loc.param("line_thickness", lineThickness, 0.005) ;

	ROS_INFO_STREAM( "x_spacing : " << xSpacing << " y_spacing : " << ySpacing << " line_thickness : " << lineThickness ) ;

	const World world(xSpacing, ySpacing, lineThickness);

	//  Ground generator parameters
	int xLines, yLines;				// number of lines
	nh_glob.param("x_lines", xLines, 4) ;
	nh_glob.param("y_lines", yLines, 4) ;


	// Declare your node's subscriptions and service clients
	ros::Subscriber RecivedPosture = nh_glob.subscribe<geometry_msgs::PoseStamped>("RobotPosture", 1, RobotPostureReceived);

    // Declare you publishers and service servers
	ros::Publisher TilesDisplay = nh_glob.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);


	WorldGenerator generator(world);

	generator.InitWorld();

	for(const auto& chunk : generator.Chunks() ) {
		TilesDisplay.publish( chunk.tiles );
	}

  ros::Rate rate(50);
  while (ros::ok()){
      ros::spinOnce();

			for(const auto& chunk : generator.Chunks() ) {
				TilesDisplay.publish( chunk.tiles );
			}


      rate.sleep();
  }
}
