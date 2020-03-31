function[Ht] = HjacDepth(pose,hParams)
% HJACDEPTH: Function to compute jacobian Ht from depth measurement
%
%   INPUTS
%       pose            Vector indicating pose of robot at last time step 3x1
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       Ht   Kx3 vector representing the Jacobian Gt evaluated at time t
%            given the robot pose at t-1 is 'pose'
%
%   NOTE: Assume differential-drive robot whose wheels turn at a constant
%         rate between sensor readings.
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #4
%   SAJAN, NAYANTHARA

mapFields               = fields(hParams);
map                     = hParams.(mapFields{1});
sensorOrigin            = hParams.(mapFields{2});
angles                  = hParams.(mapFields{3});
delta = 0.025;

depth         = depthPredict( pose                            ,hParams);
depthDelX     = depthPredict([pose(1)+delta; pose(2:3)]       ,hParams);
depthDelY     = depthPredict([pose(1); pose(2)+delta; pose(3)],hParams);
depthDelTheta = depthPredict([pose(1:2); pose(3)+delta]       ,hParams);

Ht = [(depthDelX     - depth)/delta ...
      (depthDelY     - depth)/delta ...
      (depthDelTheta - depth)/delta ];