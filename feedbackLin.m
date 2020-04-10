function[cmdV,cmdW] = feedbackLin(velX,velY,theta,epsilon)
% FEEDBACKLIN: transform Vx and Vy commands into corresponding V and w 
% commands using the feedback linearization
% 
%   [CMDV,CMDW] = FEEDBACKLIN(VELX,VELT,THETA,EPSILON) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot for given desired velocites Vx and Vy in the inertial frame.
% 
%   INPUTS
%        velX    : x-component of desired velocity in inertial frame (m/s)
%        velY    : y-component of desired velocity in inertial frame (m/s)
%        theta   : Robot's current orientation [rad]
%        epsilon : Distance from (0,0) used for feedback linearization
% 
%   OUTPUTS
%       cmdV        desired forward velocity command (m/s)
%       cmdW        desired angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   SAJAN, NAYANTHARA

VW = [1 0; 0 1/epsilon]*...
     [ cos(theta) sin(theta); -1*sin(theta) cos(theta)]*...
     [velX; velY];

cmdV = VW(1);
cmdW = VW(2); 

end
