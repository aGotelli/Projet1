% Constants defining the robot and sensor setup
% All lengths are in mm.
% You may change subSamplingFactor to 1 for an easier task in estimating
% the measurement noise, but it should be reset to 4 when you start tuning
% the Kalman filter.

% Robot characteristics

rwheel           =  21.5 ;      % Wheel radius (mm)
trackGauge       = 112   ;      % Distance between the fixed wheels (mm)
encoderRes       = 180   ;      % In dots per wheel rotation

samplingFrequency    = 20 ;
samplingPeriod       = 1/samplingFrequency ;

topRobotSpeed = 100 ;

% Homogeneous coordinates of the line detector sensors in robot frame Rm.
% One column per sensor.
            %s1   s2  ...
mSensors = [  0   0  ;
             50 -50  ;
              1   1  ] ;
nbLineDetectors = size(mSensors,2) ;                  

% Line spacing   

xSpacing = 50 ;
ySpacing = 50 ;


% ---------------------------------------------------------------
% The following are calculated from previous data. Do not modify.
% ---------------------------------------------------------------

dots2rad = (2*pi)/encoderRes ;
rad2dots = 1/dots2rad        ;

jointToCartesian = [ rwheel/2           rwheel/2          ;
                     rwheel/trackGauge -rwheel/trackGauge ] ;
                
cartesianToJoint = inv(jointToCartesian) ;                
