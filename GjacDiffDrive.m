function[Gt] = GjacDiffDrive(initPose,Ut)
% GJACDIFFDRIVE: Function to compute jacobian Gt from odometry measurement
% 
%   INPUTS
%       initPose        Vector indicating pose of robot at last time step 3x1
%       distance    last distance returned by DistanceSensorRoomba    [m]
%       angle       last angle returned by AngleSensorRoomba          [rad]      	
% 
%   OUTPUTS
%       Gt   3x3 vector representing the Jacobian Gt evaluated at time t
%            given the robot pose at t-1 is 'pose'
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

if (angle == 0)
    d = distance;
else
    d = sign(distance)*sqrt(2*(distance/angle)^2*(1-cos(angle)));
end

% find the Jacobian matrix according to the formula
Gt = [ 1 0 -d*sin(initPose(3,1) + angle/2);
       0 1  d*cos(initPose(3,1) + angle/2) ;
       0 0  1  ];
end