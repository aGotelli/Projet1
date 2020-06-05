/**
 * \file ground executable file
 * \brief collects the parameters and generates a ground
 * \author Bianca & Andrea
 * \version 0.2
 * \date 05/06/2020
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    °
 *
 * Publishes to: <BR>
 *    °
 *
 * Description
            This file is meant to collect all the parameters of the world, to
					generate a backgound for the simulation. In this way it makes the
					whole simulation more user friendly.

          Several choiches have been made following the advices of the
          Guidelines: https://github.com/isocpp/CppCoreGuidelines

          Related chapthers of the CppCoreGuidelines:
            ° All the Nl section, especially NL.16 and NL.17 but not the NL.10


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

	// Declare your node's subscriptions and service clients
	ros::Subscriber RecivedPosture = nh_glob.subscribe<geometry_msgs::PoseStamped>("RobotPosture", 1, RobotPostureReceived);

    // Declare you publishers and service servers
	ros::Publisher TilesDisplay = nh_glob.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);


	WorldGenerator generator(world);

	generator.InitWorld();

	//	Wait two seconds before publishing
	//ros::Duration(5.0).sleep();

	bool onlyOnce = true;
  ros::Rate rate(50);
  while (ros::ok()){
      ros::spinOnce();

			if(onlyOnce) {
				for(const auto& chunk : generator.Chunks() ) {
					TilesDisplay.publish( chunk->tile );
					TilesDisplay.publish( chunk->lines );
				}
			}



      rate.sleep();
  }
}
