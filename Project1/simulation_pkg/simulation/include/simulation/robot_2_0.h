#ifndef ROBOT_2_0_H
#define ROBOT_2_0_H

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

 #include "simulation/robot_base.h"


class Robot_2_0 : public RobotBase
{
public:
  //Robot_2_0() = default;

  Robot_2_0(const double _trackGauge, const double _wheelRadius) :
            trackGauge(_trackGauge),
            wheelRadius(_wheelRadius),
            RobotBase() {}

  void TwistReceived(const geometry_msgs::Twist::ConstPtr& twist) override;

  void ComputeInput() override;

  void PerformMotion() override;

  void CheckSensorStatus() override;

private:

  const double trackGauge {0.2} ;
  const double wheelRadius {0.05};
};


void Robot_2_0::TwistReceived(const geometry_msgs::Twist::ConstPtr& twist)
{
  ROS_INFO_STREAM("1");
}

void Robot_2_0::ComputeInput()
{
  ROS_INFO_STREAM("2");
}

void Robot_2_0::PerformMotion()
{
  ROS_INFO_STREAM("3");
}

void Robot_2_0::CheckSensorStatus()
{
  ROS_INFO_STREAM("4");
}




#endif //ROBOT_2_0_H
