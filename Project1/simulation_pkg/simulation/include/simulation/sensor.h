#ifndef SESNOR_H
#define SENSOR_H

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <simulation/utility.h>

// Definition of World class
class World {
public:
 World()=default;

 World(const double _xSpacing,
        const double _ySpacing) :
          xSpacing( _xSpacing ),
          ySpacing( _ySpacing ) { /*  lineWidth has default value  */
            ROS_INFO_STREAM("User-defined world only for line offset") ;
          }

 World(const double _xSpacing,
        const double _ySpacing,
          const double _lineWidth) :
            xSpacing( _xSpacing ),
            ySpacing( _ySpacing ),
            lineWidth( _lineWidth ) {
              ROS_INFO_STREAM("Completely user-defined world") ;
            }

 // Provided function to access private data
 inline const double& XSpacing() const {return xSpacing; }
 inline const double& YSpacing() const {return ySpacing; }
 inline const double& LineWidth() const {return lineWidth; }

private:

 const double xSpacing { 0.2 };
 const double ySpacing { 0.2 };
 const double lineWidth { 0.0 };

};


//Definition of Sensor class
class Sensor {
public:
 Sensor()=default;

 Sensor(const double _x, const double _y) : HCoord( Eigen::Vector3d(_x, _y, 1.0) ) {
   ROS_INFO_STREAM("User-defined sensor only for position") ;
 }

 Sensor(const double _x, const double _y, const World& _world) :
                                HCoord( Eigen::Vector3d(_x, _y, 1.0) ),
                                world(_world) {
                                  ROS_INFO_STREAM("Completely user-defined sensor") ;
                                }


 // Get the current sensor status
 inline const bool GetState(){ return state; }

 // Access the sensor position in homogeneus coordinates
 inline const Eigen::Vector3d& Coord() const { return HCoord; }

 void UpdateTransform(const geometry_msgs::Pose& robotPosture) const;

 void CheckStatus() const;


private:

  // Sensor position in homogeneus coordinates
  const Eigen::Vector3d HCoord {0.05, 0.0, 1.0};

  //  Sensor has its own istance of the world
  const World world;

  //  Sensor has information about the robot posture in term of
  //  the homogeneus transform from the reference frame (frame O indicated "o")
  //  to the moving platform frame (frame M indicated "m")
  mutable Eigen::Matrix3d oTm { Eigen::MatrixBase<Eigen::Matrix3d>::Identity() } ;

  //  The measurement
  mutable bool state {false};

};

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
  //  First, with the knowledge of the robot position obtain the coordinates
  //  of the lines surrounding it
  const int deltaX = std::floor( robot.q.x/xSpacing )*xSpacing;
  const int deltaY = std::floor( robot.q.y/ySpacing )*ySpacing;

  const int distToNextLine_x = deltaX + xSpacing;
  const int distToNextLine_y = deltaY + ySpacing;


  //  Then compute their equations in homogeneus coordinates (line equation: ax + by + c = 0.)

  //  For a "vertical" line x = a -> x - a = 0.
  //  that is in homogeneus coordinates 1*x + 0*y -a*c = 0
  const Eigen::Vector3d leftLine(0.0f, 1.0f, -deltaX);
  const Eigen::Vector3d rightLine(0.0f, 1.0f, -distToNextLine_x);

  //  For a "horizontal" line y = b -> y - b = 0.
  //  that is in homogeneus coordinates 0*x + 1*y -b*c = 0
  const Eigen::Vector3d bottomLine(1.0f, 0.0f, -deltaY);
  const Eigen::Vector3d upperLine(1.0f, 0.0f, -distToNextLine_y);

  //  Concatenate the equations
  Eigen::MatrixXd worldLines(3, 4);
  worldLines << leftLine, rightLine, bottomLine,upperLine ;


  //  Compute the dot product column-wise
  const Eigen::Vector3d overLine = HCoord.transpose()*worldLines;


  // Update the sensor states
  if(overLine.minCoeff() <= (lineWidth/2)){
      state = true;
  } else {
    state = false;
  }  


}







/*



const bool World::CheckWorldLines(const Eigen::Vector3d& HCoord, const Robot_2_0& robot) const
{

  const int nx = std::floor( robot.q.x/xSpacing )*xSpacing;
  const int ny = std::floor( robot.q.y/ySpacing )*ySpacing;

  const int next_nx = nx + xSpacing;
  const int next_ny = ny + ySpacing;

  //  In homogeneus coordinates a line has the equation: ax + by + c = 0.

  //  For a "vertical" line x = a -> x - a = 0.
  //  that is in homogeneus coordinates 1*x + 0*y -a*c = 0
  const Eigen::Vector3d leftLine(0.0f, 1.0f, -nx);
  const Eigen::Vector3d rightLine(0.0f, 1.0f, -next_nx);

  //  For a "horizontal" line y = b -> y - b = 0.
  //  that is in homogeneus coordinates 0*x + 1*y -b*c = 0
  const Eigen::Vector3d bottomLine(1.0f, 0.0f, -ny);
  const Eigen::Vector3d upperLine(1.0f, 0.0f, -next_ny);

  //  Comcatenate the line into a matrix
  Eigen::MatrixXd worldLines(3, 4);
  worldLines << leftLine, rightLine, bottomLine,upperLine ;

  const Eigen::Vector3d overLine = HCoord.transpose()*worldLines;

  return !overLine.minCoeff() ;

}

void Robot_2_0::CheckSensorStatus() const
{
  for(const auto sensor : robotSensors) {

    sensor.setState( world.CheckWorldLines( sensor.Coord(), this->q ) );
  }

}

*/

#endif //SENSOR_H
