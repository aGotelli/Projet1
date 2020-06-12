/**
 * \file robot node
 * \brief file for moving the robot
 * \author Bianca & Andrea
 * \version 0.1
 * \date 28/05/2020
 *
 * \param[in]
    robot param : track_gauge, wheel_radius, trailing_offSet, castor_arm, actuator_max_speed
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

              In this executable there is just the collection of the parameters.
            These are used to initialize the robot class and then all the
            computations are embedded in member functions.

              The idea is that, as the simulation is based on two main parts:
            the robot kinematic and the interface with the R.O.S. environment;
            they should be separated. This is done with the use of two header
            files. The class Robot_2_0 contains all the kinematic operations to
            obtain data from the simulation. Meanwhile the base class RobotBase
            provides all the R.O.S. interfaces. Moreover, it also used another
            header file in which the structure of the generalized coordinates is
            defined.
 *
 */


//Cpp
#include <simulation/robot_2_0.h>

#include <simulation/sensor.h>







int main (int argc, char** argv)
{

    // ROS Initialization
    ros::init(argc, argv, "robot_2_0");

    // Required NodeHandles
    ros::NodeHandle nh_loc("~");

    //  Get robot initial posture
    double xInit, yInit, thetaInit;
    nh_loc.param("x_init", xInit, 0.0);
    nh_loc.param("y_init", yInit, 0.0);
    nh_loc.param("theta_init", thetaInit, 0.0);

    //  Transform theta in radians
    thetaInit *= M_PI/180;

    //  Robot parameters
    double frontAxle;                             // [m]
    double wheelRadius;                           // [m]
    double trailingOffSet;                           //  [m]
    double castorArmLength;                       //  [m]
    double wMax ;                                 //  [RAD/s]
    double encodersResolution ;                   //  [dots/wheel rotation]
    double wheel1Error;                           //  [percentage]

    nh_loc.param("a", frontAxle, 0.2) ;
    nh_loc.param("wheel_radius", wheelRadius, 0.05) ;
    nh_loc.param("b", trailingOffSet, 0.4) ;
    nh_loc.param("c", castorArmLength, 0.15) ;
    nh_loc.param("actuator_max_speed", wMax, (double)10.0) ;
    nh_loc.param("encoders_resolution", encodersResolution, (double)1.0) ;
    nh_loc.param("wheel_1_error", wheel1Error, 1.01) ;

    //  Check that the passed arguments are correct and meaningfull
    if( frontAxle < 0 ||
        wheelRadius < 0 ||
        trailingOffSet < 0 ||
        castorArmLength < 0 ||
        wMax < 0 ||
        encodersResolution < 0 ) {
      ROS_ERROR_STREAM("YOU MUST USE POSITIVE VALUES FOR ROBOT PARAMETERS");

      ros::shutdown();
    }


    Robot_2_0 robot(utility::Pose2D(xInit, yInit, thetaInit),
                    frontAxle, wheelRadius, trailingOffSet,
                    castorArmLength, wMax,
                    encodersResolution, wheel1Error );

    robot.isMoving() ;

}
