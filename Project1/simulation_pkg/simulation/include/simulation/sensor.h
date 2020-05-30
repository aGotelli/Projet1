#ifndef SESNOR_H
#define SENSOR_H


#include "simulation/robot_base.h"
#include "simulation/robot_2_0.h"
#include <eigen3/Eigen/Dense> 


// Definition of World class
class World {
public:
 World()=default;

 const bool CheckWorldLines(const Eigen::Vector3d& HCoord, const Robot_2_0& robot) const;

 const bool CheckWorldLines(const Eigen::Vector3d& HCoord, const Robot_2_0::GeneralizedCorrdinates& q) const;

private:

 const double xSpacing { 0.2 };
 const double ySpacing { 0.2 };
 const double lineWidth { 0.0 };

};


//Definition of Sensor class
class Sensor {
public:
 Sensor()=default;

 Sensor(const double _x, const double _y) : HCoord( Eigen::Vector3d(_x, _y, 1.0) ) {}


 inline void setState(bool stateSensor ){ stateSensor = state; }

 inline const bool getState(){ return state; }

 inline const Eigen::Vector3d& Coord() const { return HCoord; }
 inline const std::vector<Sensor> sensorVector() const { return robotSensors; }

private:

  // Sensor parameters
  const Eigen::Vector3d HCoord {0.05, 0.0, 1.0};
  mutable bool state {false};


  friend class World;
  const World world;

  // Vector of sensor
  mutable std::vector<Sensor> robotSensors;

};






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

#endif //SENSOR_H