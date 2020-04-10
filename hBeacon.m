function[beacon] = hBeacon(robotPose,hParams)
% HBEACON: predict the beacon measurements for a robot operating
% in region with known set of beacons.
%
%   BEACON = HBEACON(ROBOTPOSE,HPARAMS) returns
%   the expected beacon measurements for a robot operating
%   in region with known set of beacons.
%
%   INPUT params[robotPose,hParams]
%
%       robotPose           3-by-1 pose vector in global coordinates (x,y,theta)
%       hParams             Struct containing
%           beaconLoc       Location of beacons Nx3 [id x y]
%           sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%           correction      [deltaDist deltaAngle] for delay correction
%
%   OUTPUTS
%       beacon              K-by-2 vector of beacon measurements [id x y](meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Lab #2
%   SAJAN, NAYANTHARA

mapFields               = fields(hParams);
map                     = hParams.(mapFields{1});
sensorOrigin            = hParams.(mapFields{2});
beaconLoc               = hParams.(mapFields{3});

N = size(map,1);
K = size(beaconLoc,1);

maxRange = 12; %Assume maximum range of sensor is 12 meters
angRange = deg2rad(2*27); %Assume angular range of sensor is -27 to 27 degrees

% Convert sensor origin to global frame
sensorOriginG = robot2global(robotPose,sensorOrigin);

% Assume sensor frame is not rotated w.r.t robot frame
sensorPoseG = [sensorOriginG robotPose(3)];

% Convert points from sensor frame to global frame
beaconLocS = cell2mat(arrayfun(@(index) global2robot(sensorPoseG,...
    [beaconLoc(index,2) beaconLoc(index,3)]),1:K,'UniformOutput',false)');

beacon = [];
for i = 1:K % For each of K beacons
    inView   = false;
    px       = beaconLocS(i,1);
    py       = beaconLocS(i,2);
    angle    = atan2(py,px);
    distance = norm([px py]);
    if (-angRange/2<angle && angle<angRange/2 && distance<maxRange)
        inView   = true;
        for j = 1:N % Calculate distance from sensor to each of N walls
            [bool,~,~,ua] = intersectPoint(sensorOriginG(1),sensorOriginG(2),...
                beaconLoc(i,2),beaconLoc(i,3),map(j,1),map(j,2),map(j,3),map(j,4));
            if(bool==1 && ua~=1)
                inView = false;
                break;
            end
        end
    end
    if(inView==true)
        beacon = [beacon; beaconLoc(i,1) beaconLocS(i,:)];
    end
end
end



