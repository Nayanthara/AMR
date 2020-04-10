function[Ht,validZt] = HjacBeacon(pose,hParams,Zt)
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
beaconLoc               = hParams.(mapFields{3});
delta = 0.001;

beaconPred         = hBeacon(pose,hParams);
beaconDelXPred     = hBeacon([pose(1)+delta; pose(2:3)],hParams);
beaconDelYPred     = hBeacon([pose(1); pose(2)+delta; pose(3)] ,hParams);
beaconDelThetaPred = hBeacon(wrap([pose(1:2); pose(3)+delta]')',hParams);
beacon             = [];
beaconDelX         = [];
beaconDelY         = [];
beaconDelTheta     = [];
validZt            = [];
for i = 1:size(Zt,1)
    if(~isempty(beaconPred)&&...
            ~isempty(beaconDelXPred)&&...
            ~isempty(beaconDelYPred)&&...
            ~isempty(beaconDelThetaPred))
        if(~isempty(find(beaconPred(:,1)==Zt(i,1)))&&...
                ~isempty(find(beaconDelXPred(:,1)==Zt(i,1)))&&...
                ~isempty(find(beaconDelYPred(:,1)==Zt(i,1)))&&...
                ~isempty(find(beaconDelThetaPred(:,1)==Zt(i,1))))
            beacon         = [beacon; beaconPred(beaconPred(:,1)==Zt(i,1),2:3)];
            beaconDelX     = [beaconDelX; beaconDelXPred(beaconDelXPred(:,1)==Zt(i,1),2:3)];
            beaconDelY     = [beaconDelY; beaconDelYPred(beaconDelYPred(:,1)==Zt(i,1),2:3)];
            beaconDelTheta = [beaconDelTheta; beaconDelThetaPred(beaconDelThetaPred(:,1)==Zt(i,1),2:3)];
            validZt        = [validZt;Zt(i,:)];
        end
    end
end
beacon          = reshape(beacon,2*size(beacon,1),1);
beaconDelX      = reshape(beaconDelX,2*size(beaconDelX,1),1);
beaconDelY      = reshape(beaconDelY,2*size(beaconDelY,1),1);
beaconDelTheta  = reshape(beaconDelTheta,2*size(beaconDelTheta,1),1);

Ht =[(beaconDelX     - beacon)/delta ...
    (beaconDelY     - beacon)/delta ...
    (beaconDelTheta - beacon)/delta ];