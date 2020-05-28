/**
 * \file
 * \brief
 * \author
 * \version 0.1
 * \date
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
 *
 */


//Cpp
#include <simulation/robot_2_0.h>


int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "robot_2_0");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_loc("~"), nh_glob;

    //  Global world parameters
    double xSpacing, ySpacing;                    // [m]
    nh_glob.param("x_spacing", xSpacing, 0.2) ;
    nh_glob.param("y_spacing", ySpacing, 0.2) ;


    //  Robot parameters
    double trackGauge;                            // [m]
    double wheelRadius;                           // [m]
    nh_loc.param("track_gauge", trackGauge, 0.2) ;
    nh_loc.param("wheel_radius", wheelRadius, 0.05) ;

    Robot_2_0 robot(trackGauge, wheelRadius) ;

    robot.isMoving() ;

}
