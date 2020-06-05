#ifndef SESNOR_H
#define SENSOR_H

/**
 * \file sensor header file
 * \brief describes and presents method to simulate the sensor
 * \author Bianca & Andrea
 * \version 0.1
 * \date 30/05/2020
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
            This file contains all the needed functons to simulates the sensors.
          the intended approach is geometrical. The sensor is expressed in
          homogeneus coordinates in the robot frame. The knowledge about the
          robot position is expressed in terms of the homogeneus transform from
          the reference frame (frame O indicated "o") to the moving platform
          frame (frame M indicated "m").

            The lines are obtained with the use of floor.
          ^  Y                            |                |
          :                         ______|________________|_______   up
          :                               |                |
          :                               |                |
          :                               |       S        |
          :                               |                |
         -:-------------------> X   ______|________________|_______   down
          :                               |                |
                                        left             right

            The sensor, assumed in the position S, has its own
          coordinates (S.x, S.y). Flooring the result of the division of
          theese coordinates by the line offsets (xSpacing and ySpacing)
          and multipling it back with the offset will give the "left" and
          "down" coordinate represented in the design. By adding the offset
          to this position, the expressions of "up" and "right" are obtained.
          The sign of the robot coordinates is taken into account. In this way
          only the nomenclature fails. In fact, the names in that case should
          be switched. However, the computation holds. It is the only thing
          that matters here. The aim is just to check in a "boolean way" and
          there is no need of determing which line is under the sensor.

            Once the expression of the lines around the sensor are known:
          "vertical" line x = left or x = right
          "horizontal" line y = down or y = up
          Their are expressed in homogeneus coordinates. A line in homogeneus
          coordinates has the expression: ax + by + c = 0. In a vector form it
          can be expressed as:

                                          | x |
                                [a  b   c]| y | = 0
                                          | 1 |

            Once the line expression in homogeneus coordinates is computed the
          distance from the sensor to these lines is the result of their dot
          product. In fact, the dot product between a point and a line is the
          length of the segment that is normal to the line and passes throught
          the point.

            The expression of the lines around the sensor are concatenated into
          a matrix. As a result, it is possible to directly compute the
          distances from them and the point (that is the sensor).
          In other words, the result is a vector which each of its elements
          represents a distance from the related line. The sensor status
          is changes accordingly with the values in this vector of distances.

            However these last ones could be positive or negative. The insight for
          the measurement comes from the minumum of the absolute values of these
          distances. The Eigen::MatrixBase has some member function which help.

            The memeber function cwiseAbs() reurns a vector containing
          the absolute value of the object calling. The other member function
          minCoeff() returns the minimum among the coefficients of the object
          calling. The minimum will be zero only when the sensor is exactly
          on the line. This last situation is quite unlikely to happen.
          So the distance is compared to a threshold (non-zero).

            Moreover, in a practical application, all the lines have a finite
          thickness. In order to be consistent, if the line has a thickness
          of 1 cm, then the status of the sensor should output a measurement
          if its distance from the line is less then half the thickness
 *
 */



// ROS
#include <ros/ros.h>

#include "simulation/utility.h"
#include "simulation/world.h"


#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>



//Definition of Sensor class
class Sensor {
public:
 Sensor()=default;

 Sensor(const double _x, const double _y) : HCoord( Eigen::Vector3d(_x, _y, 1.0) ) {
   ROS_DEBUG_STREAM("User-defined sensor only for position") ;
 }

 Sensor(const double _x, const double _y, const World& _world) :
                                HCoord( Eigen::Vector3d(_x, _y, 1.0) ),
                                world(_world) {
                                  ROS_DEBUG_STREAM("Completely user-defined sensor") ;
                                }


 // Get the current sensor status
 inline const bool GetState() const { return state; }

 // Access the sensor position in homogeneus coordinates
 inline const Eigen::Vector3d& Coord() const { return HCoord; }


 // Function to update the transformation matrix
 void UpdateTransform(const geometry_msgs::Pose& robotPosture) const;

 // Function to check the status of the sensors
 void CheckStatus() const;


 // Function to get the sensor position in the absolute frame
 inline const utility::Pose2D AbsolutePosition() const;

 inline const World ItsWorld() const {return world;}



private:

  // Sensor position in homogeneus coordinates
  const Eigen::Vector3d HCoord {0.1, 0.0, 1.0};

  //  Sensor has its own istance of the world
  const World world;

  //  Homogeneus transform from robot to reference frame
  mutable Eigen::Matrix3d oTm { Eigen::MatrixBase<Eigen::Matrix3d>::Identity() } ;

  //  The measurement
  mutable bool state {false};

};



///////////////////////////////////////////~ FUNCTIONS DECLARATION ~\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


void Sensor::UpdateTransform(const geometry_msgs::Pose& robotPosture) const
{
  //  First convert from quaternion to Euler Angles
  const utility::EulerAngles eulers = utility::ToEulerAngles( robotPosture.orientation ) ;

  oTm <<  cos( eulers.yaw ), -sin( eulers.yaw ), robotPosture.position.x  ,
          sin( eulers.yaw ),  cos( eulers.yaw ), robotPosture.position.y  ,
                  0        ,          0        ,            1             ;

}


void Sensor::CheckStatus() const
{

  //  First, with the knowledge of the robot position obtain the sensor position
  const auto sensorPosition = this->AbsolutePosition() ;

  //  With the knowledge of the sensor position obtain the coordinates
  //  of the lines surrounding it
  const double left = std::floor( sensorPosition.x/world.XSpacing() )*world.XSpacing();
  const double down = std::floor( sensorPosition.y/world.YSpacing() )*world.YSpacing();

  const double right = left + world.XSpacing() ;
  const double up    = down + world.YSpacing() ;

  //  For a "vertical" line x = a -> x - a = 0.
  //  that is in homogeneus coordinates 1*x + 0*y -a*c = 0
  const Eigen::Vector3d leftLine(1.0f, 0.0f, -left);
  const Eigen::Vector3d rightLine(1.0f, 0.0f, -right);

  //  For a "horizontal" line y = b -> y - b = 0.
  //  that is in homogeneus coordinates 0*x + 1*y -b*c = 0
  const Eigen::Vector3d bottomLine(0.0f, 1.0f, -down);
  const Eigen::Vector3d upperLine(0.0f, 1.0f, -up);

  //  Concatenate the equations
  Eigen::MatrixXd worldLines(3, 4);
  worldLines << leftLine, rightLine, bottomLine, upperLine ;

  //  Compute the sensor position w.r.t. the reference frame
  const Eigen::Vector3d oHCoord = oTm*HCoord;

  //  Compute the dot product column-wise.
  const Eigen::VectorXd overLine = oHCoord.transpose()*worldLines;

  //  Update the sensor status using the knowledge of the computed distances.
  if( overLine.cwiseAbs().minCoeff() <= world.LineThickness()/2 ) {
      state = true;
  } else {
      state = false;
  }

}


const utility::Pose2D Sensor::AbsolutePosition() const
{
  const Eigen::Vector3d oHCoord = oTm*HCoord;

  return utility::Pose2D( oHCoord[0], oHCoord[1] ) ;
}


class RobotSensors {
public:
  RobotSensors()=default;

  void AddSensor(const Sensor& newSensor );

  inline const std::vector<Sensor>& All() const {return sensors; }

private:
  std::vector<Sensor> sensors;
};


void RobotSensors::AddSensor(const Sensor& newSensor)
{
  if( sensors.empty() ) {
    //  Directly add the new member
    sensors.push_back( newSensor );
  } else {
      //  Check that the two instances of the world are the same
      if( sensors[0].ItsWorld() ==  newSensor.ItsWorld() )
          sensors.push_back( newSensor );
      else{
          //  If the two instances are not the same, the simualtion results
          //  are inconsistent. The node is shutted down while trowing an error.
          ROS_ERROR_STREAM("Two different istances of the world detected. The node" << ros::this_node::getName() << "is shutted down");
          ros::shutdown();
      }
  }


}



#endif //SENSOR_H
