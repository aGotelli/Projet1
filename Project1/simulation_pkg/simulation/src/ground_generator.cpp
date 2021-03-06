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
				generate a background for the simulation. In this way, it makes the
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

void RobotPostureReceived(const geometry_msgs::Pose::ConstPtr& _robotPosture)
{
	robotPosture = (*_robotPosture);
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


	const World world(xSpacing, ySpacing, lineThickness);

	// Declare your node's subscriptions and service clients
	ros::Subscriber RecivedPosture = nh_glob.subscribe<geometry_msgs::Pose>("RobotPosture", 1, RobotPostureReceived);

    // Declare your publishers and service servers
	ros::Publisher TilesDisplay = nh_glob.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);


	WorldGenerator generator(world);

	generator.InitWorld();

	bool onlyOnce = true;

  ros::Rate rate(50);
	double t0 = ros::Time::now().toSec();

  while (ros::ok()){
      ros::spinOnce();

			generator.ChuckBelonging(robotPosture);

			for(const auto& chunk : generator.Chunks() ) {
				TilesDisplay.publish( chunk->tile );
				TilesDisplay.publish( chunk->lines );
			}

			//	Ensure the application to have correctly started and that rviz has
			//	obtained the ground. Then delete the chunk to save memory
			if( (ros::Time::now().toSec() - t0) > 5 )
				generator.ClearChunks();


      rate.sleep();
  }
}
