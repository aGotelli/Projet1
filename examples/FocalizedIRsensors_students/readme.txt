This folder contains a simple simulation of a (2,0) robot equipped 
with focalized IR sensors to detect lines on the ground.

The aim of this program is to provide simulation data to test various 
localization algorithms using this setup.

RobotAndSensorDefinition contains the definition of the geometrical 
parameters of the robot, the position of the infrared sensors with respect
to the robot frame and the sampling frequency.

CreateRobotTraj is used first to create a path using the mouse. The path
is created using splines. Left click creates a new via point. Right click
creates last point. The file produced should be stored in the "data" folder.

SimulateSensors then uses the robot parameters and sampling frequency to
calculate the proprioceptive and exteroceptive sensor outputs along a path
created with CreateRobotPath. The data file produced should be stored in 
the "data" folder and used for the development of localization algorithms.

