function[] = motionControl_lab(filter,dataStore)
% MOTIONCONTROL: Function to move a robot in a map while responding to bump
% sensor and estimating pose using differnt localization methods
%
%  To switch between localization methods, comment/uncomment the 'filter'
%  variable (lines 57 to 59 )
%  choose filter 'ekfgps' for EKF with GPS measurements
%  choose filter 'ekfdepth' for EKF with depth measurements
%  choose filter 'pf' for particle filter
%  All variables are descriptively named. Initialization values may be changed in the
%  initialization section for each filter.
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #4
%   SAJAN, NAYANTHARA

%*************************************%
%           CHOOSE FILTER             %
%*************************************%
%                                     %
%     filter   ='ekfgps';
%     filter   ='ekfdepth';
%     filter   ='pf';
%                                     %
%*************************************%

tic
noRobotCount            = 0;
sensorOrigin            = [0.13 0];
angles                  = linspace(deg2rad(27),-deg2rad(27),9)';

% Load map
mapStruct               = load('labmap.mat');
mapFields               = fields(mapStruct);
map                     = mapStruct.(mapFields{1});

% Read and store sensor data
trueInitialPose         = wrap(dataStore.truthPose(1,2:end));
dataStore.deadReck      = [toc trueInitialPose];

if startsWith(filter,'ekf')
    % Initialize variables for EKF
    R                       = 0.01*eye(3);
    Qval                    = 0.01;
    Q                       = Qval*eye(9);
    dataStore.ekfMu         = [toc trueInitialPose];
    dataStore.ekfSigma      = [toc 0.05 0 0 0 0.05 0 0 0 0.1];
    dataStore.GPS           = [];
    beaconLoc               = load('labBeaconCorrected.mat').beaconLoc;
else
    % Initialize variables for PF
    setSize = 50;
    particleSet = [];
    weights = (1/setSize)*ones(1,setSize);
    for i=1:setSize
        x = trueInitialPose(1);
        y = trueInitialPose(2);
        theta = trueInitialPose(3);
        particle = [x y theta];
        particleSet =[particleSet; particle];
    end
    dataStore.particles = [toc reshape(particleSet,1,3*setSize)];
end

%--------------------------------------------------------------------------
% CONTROL LOOP
%--------------------------------------------------------------------------
i=2;
j=1;
while i <= size(dataStore.truthPose,1)
    
    % Read and store sensor data
    truePose           = wrap(dataStore.truthPose(i,[2 3 4]))';
    distance           = dataStore.odometry(i,2);
    angle              = dataStore.odometry(i,3);
    
    % Calculate pose using dead reckoning
    deadReck           = integrateOdom(dataStore.deadReck(end,2:4)',[distance angle]);
    dataStore.deadReck = [dataStore.deadReck; [toc deadReck']];
    
    % LOCALIZATION (where is the robot?)
    %-----------------------------------
    gFunc    = 'integrateOdom';
    gJacFunc = 'GjacDiffDrive';
    Ut       = [distance angle];
    
    if startsWith(filter,'ekf')
        if strcmp(filter,'ekfgps')
            %=========================================================
            %   EKF WITH GPS
            %=========================================================
            GPS                = wrap(truePose+sqrt(0.01)*randn(size(truePose))')';
            dataStore.GPS      = [dataStore.GPS; [toc GPS']];
            validIndices       = find(~isnan(GPS));
            hFunc              = 'hGPS';
            hJacFunc           = 'HjacGPS';
            hParams            = struct();
            [mu,sigma]         = EKF(dataStore.ekfMu(i-1,2:4)',reshape(dataStore.ekfSigma(i-1,2:end),3,3),...
                gFunc,gJacFunc,Ut,hFunc,hJacFunc,hParams,GPS,validIndices,R,Q,Inf);
            %=========================================================
        end
        if strcmp(filter,'ekfdepth')
            %=========================================================
            %   EKF WITH DEPTH
            %=========================================================
            hFunc        = 'depthPredict';
            hJacFunc     = 'HjacDepth';
            Zt           =  dataStore.rsdepth(i,3:end)';
            validIndices = find(~isnan(Zt(:,1))& Zt(:,1)> 0.19);
            time         = dataStore.odometry(i,1)- dataStore.odometry(i-1,1);
            delay        = dataStore.rsdepth(i,2);
            correction   = (Ut./time).*delay;
            hParams      = struct('map',map,'sensorOrigin',sensorOrigin,'angles',angles,'correction',correction);
            [mu,sigma]   = EKF(dataStore.ekfMu(i-1,2:4)',reshape(dataStore.ekfSigma(i-1,2:end),3,3),...
                gFunc,gJacFunc,Ut,hFunc,hJacFunc,hParams,Zt,validIndices,R,Q,0.5);
            %=========================================================
        end
        if strcmp(filter,'ekfbeacon')
            %=========================================================
            %   EKF WITH BEACON
            %=========================================================
            if j>size(dataStore.beacon,1)
                skipUpdate = true;
                Zt = [];
            else
                if ((dataStore.beacon(j,1)-dataStore.beacon(j,2))> dataStore.truthPose(i,1))
                    skipUpdate = true;
                    Zt = [];
                else
                    skipUpdate = false;
                    Zt = dataStore.beacon(dataStore.beacon(:,1)==dataStore.beacon(j,1),3:5);
                    j = j+size(Zt,1);
                end
            end
            hParams      = struct('map',map,'sensorOrigin',sensorOrigin,'beaconLoc',beaconLoc);
            [mu,sigma]   = EKFBeacon(dataStore.ekfMu(i-1,2:4)',reshape(dataStore.ekfSigma(i-1,2:end),3,3),...
                Ut,hParams,Zt,R,Qval,skipUpdate);
            %=========================================================
        end
        dataStore.ekfMu     = [dataStore.ekfMu; [toc mu']];
        dataStore.ekfSigma  = [dataStore.ekfSigma; [toc reshape(sigma,1,9)]];
    else
        %=============================================================
        % PARTICLE FILTER WITH DEPTH
        %=============================================================
        % Set parameters
        hFunc    = 'depthPredict';
        time         = dataStore.odometry(i,1)- dataStore.odometry(i-1,1);
        delay        = dataStore.rsdepth(i,2);
        correction   = (Ut./time).*delay;
        hParams      = struct('map',map,'sensorOrigin',sensorOrigin,'angles',angles,'correction',correction);
        % Call particle filter function
        [particleSet]       = PF(particleSet,gFunc,hFunc,hParams,Ut,dataStore.rsdepth(i,3:end)',map);
        dataStore.particles = [dataStore.particles; toc reshape(particleSet,1,3*setSize)];
        %=============================================================
    end
    i = i+1;
end
%--------------------------------------------------------------------------
% PLOT GRAPHS
%--------------------------------------------------------------------------
if strcmp(filter,'ekfgps')
    plotEKFTrajectories(map,'GPS',dataStore);
end
if strcmp(filter,'ekfdepth')
    plotEKFTrajectories(map,'Depth',dataStore);
end
if strcmp(filter,'ekfbeacon')
    plotEKFTrajectories(map,'Beacon',dataStore);
end
if strcmp(filter,'pf')
    plotParticles(map,setSize,dataStore);
end

end

