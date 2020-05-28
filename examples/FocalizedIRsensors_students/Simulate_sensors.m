RobotAndSensorDefinition ;

%Load the data file
dataFile = uigetfile('data/*.*','Select data file') ;
if isunix 
    eval(['load data/' , dataFile]) ;
else
    eval(['load data\' , dataFile]) ;
end

% Resample at the desired frequency (should be lower than the frequency 
% obtained using the simulation, say ten times less).
% Data to resample: qRight, qleft, xq, yq, theta
totalTime = tq(length(tq)) ;
nbSamples = floor(totalTime/samplingPeriod) ;
treal     = [0:nbSamples]*samplingPeriod ; 
treal     = treal.' ;
qR        = interp1 (tq, qRight, treal) ;
qL        = interp1 (tq, qLeft , treal) ;
xreal     = interp1 (tq, xq    , treal) ;
yreal     = interp1 (tq, yq    , treal) ;
thetareal = interp1 (tq, thetaq, treal) ;

% Apply quantization noise on encoder values based on resolution
qR = round(qR*rad2dots)*dots2rad ;
qL = round(qL*rad2dots)*dots2rad ;

% Now simulate the line detector state along the path, taking into 
% account their location with respect to the robot frame.
% Principle: The ground is assumed to be a checkerboard floor. The sensor
% state is 1 if the sensor is above a white square, 0 otherwise.
% Let xs,ys be the absolute coordinates of the sensor and xSpacing,ySpacing
% the x and y dimensions of the rectangles of the checkerboard floor.
% Assuming the rectangle [0,xSpacing]x[0,ySpacing] is white, then:
% if floor(xs/xSpacing) + floor(ys/xSpacing) is even, the sensor is above 
% a white square, otherwise it's above a black square.

% This is just for checking graphically with two sensors. Not general code.
%nbWhites = [0 0] ;
%nbBlacks = [0 0];

sensorState = zeros( length(treal) , nbLineDetectors ) ;
for i = 1 : nbSamples
    for j = 1 : nbLineDetectors
        oTm = [ cos(thetareal(i))  ,  -sin(thetareal(i))  ,  xreal(i)  ;
                sin(thetareal(i))  ,   cos(thetareal(i))  ,  yreal(i)  ; 
                      0        ,         0        ,    1   ] ;
        oSensor = oTm * mSensors(:,j) ;
        xs = oSensor(1) ;
        ys = oSensor(2) ;
        sensorState(i,j) = ~rem( floor(xs/xSpacing)+floor(ys/ySpacing) , 2 ) ;
    end
end

save simu dots2rad rad2dots rwheel trackGauge topRobotSpeed ...
          mSensors xSpacing ySpacing ...
          treal xreal yreal thetareal qR qL sensorState ;
