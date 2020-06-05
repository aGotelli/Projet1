#ifndef ROBOT_2_0_GENERALIZEDCOORD_H
#define ROBOT_2_0_GENERALIZEDCOORD_H

/**
 * \file robot_2_0 generalized coordinates
 * \brief generalized coordinates of the robot
 * \author Bianca & Andrea
 * \version 0.2
 * \date 01/06/2020
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
            This file cointains all the functions and methods implemented to define the kinematic
          of the robot. All the computations used the generalized coordinates as it is a powerfull
          structure, perfectly suitable for type of calculations. In this way the robot file
          can remail light and easy to read throught. All the "complexities" are here.

            To avoid memory leaks, the use of smart pointers is strongly recommended. They are
          used in functions where we need to pass ownership of an object. (see guidelines)


          Several choiches have been made following the advices of the
          Guidelines: https://github.com/isocpp/CppCoreGuidelines

          Related chapthers of the CppCoreGuidelines:
            ° Con.1, Con.2
            ° All the Nl section, especially NL.16 and NL.17 but not the NL.10
            ° R.20, R.21, R.23, R.30

 *
 */


#include "simulation/robot_base.h"
#include <eigen3/Eigen/Dense>  //  usefull for matrix vectors operations
#include <memory>


namespace robot_2_0 {

  struct GeneralizedCoordinates  {

    GeneralizedCoordinates()=default;

    //  Copy Constructor
    GeneralizedCoordinates(GeneralizedCoordinates& other);

    //  Specific Constructor
    GeneralizedCoordinates(const double _x, const double _y, const double _theta,
                            const double _beta_3c, const double _phi_1f,
                            const double _phi_2f, const double _phi_3c);

    //  Initial posture Constructor
    GeneralizedCoordinates(const double _x, const double _y, const double _theta);

    ~GeneralizedCoordinates() {/* no new objects to delete */}

    //  Move Constructor
    GeneralizedCoordinates(GeneralizedCoordinates&& other) noexcept;

    //  Move Operator
    GeneralizedCoordinates& operator=(GeneralizedCoordinates&& other) noexcept;

    //  Operator Equal
    GeneralizedCoordinates operator=(const GeneralizedCoordinates& equal);

    //  Operator Plus
    GeneralizedCoordinates operator+(const GeneralizedCoordinates& addendum);

    //  Operator equal to an Eigen Vector
    GeneralizedCoordinates operator=(const Eigen::VectorXd& result);

    //  Operator equal to an Eigen Vector
    GeneralizedCoordinates operator+(const Eigen::VectorXd& result);

    //  Performing the computation of the dispacement
    std::unique_ptr<GeneralizedCoordinates> Integrate(const ros::Duration& timeElapsed);


    //  Robot position and orientation
    double x{0.0}, y{0.0}, theta{0.0};

    //  Angle of the castor joint
    double beta_3c{0.0};

    //  Wheels angles of rotation
    double phi_1f{0.0}, phi_2f{0.0}, phi_3c{0.0};


  };

  //  Constructor for passing each member
  GeneralizedCoordinates::GeneralizedCoordinates(GeneralizedCoordinates& other) :
                            GeneralizedCoordinates(other.x, other.y, other.theta, other.beta_3c,
                                                      other.phi_1f, other.phi_2f, other.phi_3c ) {}

  //  Specific Constructor
  GeneralizedCoordinates::GeneralizedCoordinates(const double _x, const double _y, const double _theta,
                                                  const double _beta_3c, const double _phi_1f,
                                                  const double _phi_2f, const double _phi_3c ) :
                                                  x(_x), y(_y), theta(_theta), beta_3c(_beta_3c),
                                                  phi_1f(_phi_1f), phi_2f(_phi_2f), phi_3c(_phi_3c)  {}


  //  Initial posture Constructor
  GeneralizedCoordinates::GeneralizedCoordinates(const double _x, const double _y, const double _theta):
                                                                            x(_x), y(_y), theta(_theta) {}



  //  Move Constructor
  GeneralizedCoordinates::GeneralizedCoordinates(GeneralizedCoordinates&& other) noexcept :
                    x( other.x ),
                    y( other.y ),
                    theta( other.theta ),
                    beta_3c( other.beta_3c),
                    phi_1f( other.phi_1f ),
                    phi_2f( other.phi_2f),
                    phi_3c( other.phi_3c)  {}


  //  Move Operator
  GeneralizedCoordinates& GeneralizedCoordinates::operator=(GeneralizedCoordinates&& other) noexcept
  {
    std::swap(x, other.x);
    std::swap(y, other.y);
    std::swap(theta, other.theta);
    std::swap(beta_3c, other.beta_3c);
    std::swap(phi_1f, other.phi_1f);
    std::swap(phi_2f, other.phi_2f);
    std::swap(phi_3c, other.phi_3c);

    return *this;
  }


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



  GeneralizedCoordinates GeneralizedCoordinates::operator+(const Eigen::VectorXd& result)
  {

    this->x       += result(0) ;
    this->y       += result(1) ;
    this->theta   = utility::LimitAngle( this->theta    + result(2)  ) ;
    this->beta_3c = utility::LimitAngle( this->beta_3c  + result(3)  ) ;
    this->phi_1f  += result(4) ;
    this->phi_2f  += result(5) ;
    this->phi_3c  += result(6) ;

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
    this->phi_1f  += addendum.phi_1f ;
    this->phi_2f  += addendum.phi_2f ;
    this->phi_3c  += addendum.phi_3c ;

    return (*this) ;
  }



  std::unique_ptr<GeneralizedCoordinates> GeneralizedCoordinates::Integrate(const ros::Duration& timeElapsed)
  {

    return std::make_unique<GeneralizedCoordinates>(x       *timeElapsed.toSec() ,
                                                    y       *timeElapsed.toSec() ,
                                                    theta   *timeElapsed.toSec() ,
                                                    beta_3c *timeElapsed.toSec() ,
                                                    phi_1f  *timeElapsed.toSec() ,
                                                    phi_2f  *timeElapsed.toSec() ,
                                                    phi_3c  *timeElapsed.toSec() ) ;
  }


} //  End of the namespace





#endif //ROBOT_2_0_GENERALIZEDCOORD_H
