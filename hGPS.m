function[pose] = hGPS(initPose,~)
% HGPS: Measurement function for overhead localization.
%
%   OUTPUTS
%       pose   3x1 vector indicating estimated pose of robot
%
%   NOTE: Assume differential-drive robot whose wheels turn at a constant
%         rate between sensor readings.
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #4
%   SAJAN, NAYANTHARA

pose = wrap(initPose')';

end