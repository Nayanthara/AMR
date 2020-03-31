function[Ht] = HjacGPS(initPose,~)
% HJACGPS: Function to compute jacobian Ht for GPS measurement
% 
%   OUTPUTS
%       Ht   3x3 vector representing the Jacobian Ht for GPS measurements
% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #4
%   SAJAN, NAYANTHARA

Ht = eye(size(initPose,1));

end