function[pose] = integrateOdom(initPose,Ut)
% INTEGRATEODOM: Function to integrate the robot odometry from
% initial pose, delta distance and delta angle.
%
%   dataStore = INTEGRATEODOM(CreatePort,DistPort,TagPort,tagNum,maxTime,pose,delta) runs
%
%   INPUTS
%       initPose    Vector indicating initial pose of robot     3x1
%       distance    Distance returned by DistanceSensorRoomba   [m]
%       angle       Angle returned by AngleSensorRoomba        	[rad]
%
%   OUTPUTS
%       pose   3x1 vector indicating estimated final pose of robot
%
%   NOTE: Assume differential-drive robot whose wheels turn at a constant
%         rate between sensor readings.
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #4
%   SAJAN, NAYANTHARA

distance = Ut(1);
angle    = Ut(2);
x        = initPose(1,1);
y        = initPose(2,1);
theta    = initPose(3,1);

if angle~=0
    radius   = distance/angle;
    chordLen = 2*radius*sin(angle/2);
else
    chordLen = distance;
end

x     = x + chordLen*cos(theta+ angle/2);
y     = y + chordLen*sin(theta+ angle/2);
theta = theta + angle;
pose  = [x; y; theta];

end

