function[depth] = depthPredict(robotPose,hParams)
% DEPTHPREDICT: predict the depth measurements for a robot operating
% in a known map.
%
%   DEPTH = DEPTHPREDICT(ROBOTPOSE,MAP,SENSORORIGIN,ANGLES) returns
%   the expected depth measurements for a robot operating in a known
%   map.
%
%   INPUT params[robotPose,map,sensorOrigin,angles]
%
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame. [rad]
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #4
%   SAJAN, NAYANTHARA

mapFields               = fields(hParams);
map                     = hParams.(mapFields{1});
sensorOrigin            = hParams.(mapFields{2});
angles                  = hParams.(mapFields{3});
correction              = hParams.(mapFields{4});

N = size(map,1);
K = size(angles,1);

maxRange = 12; %Assume maximum range of sensor is 3 meters
slopes = tan(angles);

%delay correction
robotPose = integrateOdom(robotPose,correction);

% In the sensor-fixed frame, the equation of line for measurement of range
% is y = mx since it passes through origin. The second point to be used in
% the intersectPoint function can be calculated using maxRange and slopes.

x2 = maxRange./sqrt(1+slopes.^2);
y2 = slopes .* x2;

% Convert sensor origin to global frame
sensorOriginG = robot2global(robotPose,sensorOrigin);

% Assume sensor frame is not rotated w.r.t robot frame
sensorPoseG = [sensorOriginG robotPose(3)];

% Convert points from sensor frame to global frame
p2 = cell2mat(arrayfun(@(index) robot2global(sensorPoseG,[x2(index) y2(index)]),...
     1:K,'UniformOutput',false)');

range = [];
for i = 1:K % For each of K orientations of the sensor
    distance = [];
    for j = 1:N % Calculate distance from sensor to each of N walls
        [bool,~,~,ua] = intersectPoint(sensorOriginG(1),sensorOriginG(2),p2(i,1),p2(i,2),...
                        map(j,1),map(j,2),map(j,3),map(j,4));
        if bool==1
            distance = [distance; ua*maxRange];
        else
            distance = [distance; maxRange];
        end
    end
    range = [range; min(distance)];

end

depth = range .* cos(angles);

end



