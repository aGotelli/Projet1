#ifndef ROBOT_2_0_CHARACTERISTICS_H
#define ROBOT_2_0_CHARACTERISTICS_H

/**
 * \file robot 2_0 file
 * \brief contains the model of the robot
 * \author Bianca & Andrea
 * \version 0.1
 * \date 28/05/2020
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
            This file contains all the functions related to the simulation of the robot motion.
          The aim is to get information regarding the robot position and velocity and regarding
          the state of the sensors. So the kinematic of the robot is defined by the functions
          and method implemented here.

            First the imput is computed from the twist message, and the max speed
          of the wheels in ensured to both. Then with the knowledge of the input,
          the matrix which represents the kinematic model is updated and used to
          obain the derivative of the generalized robot coordinates.

            Once the derivative of the robot generalized coordinates is computed
          it is necessary to perform and integration over the time elapsed from
          the last one (so it is more like to compute a displacement). Then the
          displacement is added to the current value so all the coordinates are
          updated.

          Several coiches have been made following the advices of the
          Guidelines: https://github.com/isocpp/CppCoreGuidelines

          Related chapthers of the CppCoreGuidelines:
            ° Con.1, Con.2
            ° All the Nl section, especially NL.16 and NL.17 but not the NL.10

 *
 */


#include "simulation/robot_base.h"
#include <eigen3/Eigen/Dense>  //  usefull for matrix vectors operations


namespace robot_2_0 {

  struct GeneralizedCoordinates  {

    GeneralizedCoordinates()=default;

    GeneralizedCoordinates(GeneralizedCoordinates& other) :
    GeneralizedCoordinates(other.x, other.y, other.theta, other.phi_1f,
                                  other.phi_2f, other.phi_3c, other.beta_3c) {}

    GeneralizedCoordinates(const double _x, const double _y, const double _theta,
                            const double _phi_1f, const double _phi_2f,
                            const double _phi_3c, const double _beta_3c) :
                            x(_x), y(_y), theta(_theta), phi_1f(_phi_1f),
                            phi_2f(_phi_2f), beta_3c(_beta_3c) {}

    ~GeneralizedCoordinates() {/* no new objects to delete */}

    //GeneralizedCoordinates(GeneralizedCoordinates&& other)=delete;

    //GeneralizedCoordinates& operator=(GeneralizedCoordinates&& other)=delete;


    GeneralizedCoordinates operator=(const GeneralizedCoordinates& equal);
    //============================ AGGIUNGI REGOLA DEI CINQUE ==================
    GeneralizedCoordinates operator=(const Eigen::VectorXd& result);

    GeneralizedCoordinates operator+(const GeneralizedCoordinates& addendum);


    GeneralizedCoordinates Integrate(const ros::Duration& timeElapsed);

    double x{0.0}, y{0.0}, theta{0.0};

    double phi_1f{0.0}, phi_2f{0.0}, phi_3c{0.0};

    double beta_3c{0.0};

  };




  // Genralized coordinates' functions and operatos
  GeneralizedCoordinates GeneralizedCoordinates::operator=(const Eigen::VectorXd& result)
  {

    this->x       = result(0) ;
    this->y       = result(1) ;
    this->theta   = result(2) ;
    this->beta_3c = result(3) ;
    this->phi_1f  = result(4) ;
    this->phi_2f  = result(5) ;
    this->phi_3c  = result(6) ;

    return (*this) ;
  }



  GeneralizedCoordinates GeneralizedCoordinates::operator=(const GeneralizedCoordinates& equal)
  {

    this->x       = equal.x       ;
    this->y       = equal.y       ;
    this->theta   = equal.theta   ;
    this->beta_3c = equal.beta_3c ;
    this->phi_1f  = equal.phi_1f  ;
    this->phi_2f  = equal.phi_2f  ;
    this->phi_3c  = equal.phi_3c  ;

    return (*this) ;
  }



  GeneralizedCoordinates GeneralizedCoordinates::operator+(const GeneralizedCoordinates& addendum)
  {
    this->x += addendum.x ;
    this->y += addendum.y ;
    this->theta   = utility::LimitAngle( this->theta    + addendum.theta    ) ;
    this->beta_3c = utility::LimitAngle( this->beta_3c  + addendum.beta_3c  ) ;
    this->phi_1f  = utility::LimitAngle( this->phi_1f   + addendum.phi_1f   ) ;
    this->phi_2f  = utility::LimitAngle( this->phi_2f   + addendum.phi_2f   ) ;
    this->phi_3c  = utility::LimitAngle( this->phi_3c   + addendum.phi_3c   ) ;

    return (*this) ;
  }



  GeneralizedCoordinates GeneralizedCoordinates::Integrate(const ros::Duration& timeElapsed)
  {
    GeneralizedCoordinates delta;

    //  Function called from a q_dot object, actually x, y, theta represents velocities
    delta.x       = x       *timeElapsed.toSec() ;
    delta.y       = y       *timeElapsed.toSec() ;
    delta.theta   = theta   *timeElapsed.toSec() ;
    delta.beta_3c = beta_3c *timeElapsed.toSec() ;
    delta.phi_1f  = phi_1f  *timeElapsed.toSec() ;
    delta.phi_2f  = phi_2f  *timeElapsed.toSec() ;
    delta.phi_3c  = phi_3c  *timeElapsed.toSec() ;

    return delta;
  }

}





#endif //ROBOT_2_0_CHARACTERISTICS_H
