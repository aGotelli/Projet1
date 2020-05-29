/**
 * \file robot node
 * \brief file for moving the robot
 * \author Bianca & Andrea
 * \version 0.1
 * \date 28/05/2020
 *
 * \param[in]
    world param : xSpacing, ySpacing
    robot param : trackGauge, wheelRadius
 *
 * Subscribes to: <BR>
 *    °
 *
 * Publishes to: <BR>
 *    °
 *
 * Description
            The aim of this file is to move the robot. So it contains all the 
            information regarding the world and the robot.
 *
 */


//Cpp
#include <simulation/robot_2_0.h>


int main (int argc, char** argv)
{

	  // ROS Initialization
    ros::init(argc, argv, "robot_2_0");

    //  Required NodeHandles
    ros::NodeHandle nh_loc("~"), nh_glob;

    //  Global world parameters
    double xSpacing, ySpacing;                    // [m]
    nh_glob.param("x_spacing", xSpacing, 0.2) ;
    nh_glob.param("y_spacing", ySpacing, 0.2) ;


    //  Robot parameters
    double trackGauge;                            // [m]
    double wheelRadius;                           // [m]
    double jointOffSet;                           //  [m]
    double castorArmLength;                       //  [m]
    double wMax ;                                 //  [RAD/s]
    nh_loc.param("track_gauge", trackGauge, 0.2) ;
    nh_loc.param("wheel_radius", wheelRadius, 0.05) ;
    nh_loc.param("joint_offset", jointOffSet, 0.4) ;
    nh_loc.param("castor_arm", castorArmLength, 0.08) ;
    nh_loc.param("actuator_max_speed", wMax, (double)10.0) ;

    Robot_2_0 robot(trackGauge, wheelRadius, jointOffSet, castorArmLength, wMax);

    robot.isMoving() ;

}
