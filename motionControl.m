function[dataStore] = motionControl(Robot,maxTime)
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

%--------------------------------------------------------------------------
% INITIALIZE VARIABLES
%--------------------------------------------------------------------------
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 60;
end

try
    % When running with the real robot, we need to define the appropriate
    % ports. This will fail when NOT connected to a physical robot
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
    'odometry', [], ...
    'rsdepth', [], ...
    'bump', [], ...
    'beacon', [],...
    'deadReck', [], ...
    'ekfMu', [], ...
    'ekfSigma', [], ...
    'GPS', [], ...
    'particles',[],...
    'errorNorm',[]);
%*************************************%
%           CHOOSE FILTER             %
%*************************************%
%                                     %              
%     filter   ='ekfgps'; 
    filter   ='ekfdepth';
%     filter   ='pf';                   
%                                     %
%*************************************%

tic
noRobotCount            = 0;
sensorOrigin            = [0.13 0];
angles                  = linspace(deg2rad(27),-deg2rad(27),9)';

% Load map
mapStruct               = load('cornerMap.mat');
mapFields               = fields(mapStruct);
map                     = mapStruct.(mapFields{1});

% Read and store sensor data
[noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
bump                    = dataStore.bump(end,[2 3 7] );
trueInitialPose         = dataStore.truthPose(end,2:end);
dataStore.deadReck      = [toc trueInitialPose];

if startsWith(filter,'ekf')
    % Initialize variables for EKF
    R                       = 0.01*eye(3);
    Q                       = 0.001*eye(9);
    dataStore.ekfMu         = [toc trueInitialPose];
    dataStore.ekfSigma      = [toc 2 0 0 0 2 0 0 0 0.1];
else
    % Initialize variables for PF
    setSize = 20;
    particleSet = [];
    weights = (1/setSize)*ones(1,setSize);
    for i=1:setSize
        x = -5 + 5*rand();
        y = -5 + 10*rand();
        theta = -0.2+0.4*rand();
        particle = [x y theta];
        particleSet =[particleSet; particle];
    end
    dataStore.particles = [toc reshape(particleSet,1,3*setSize)];
end

%--------------------------------------------------------------------------
% CONTROL LOOP
%--------------------------------------------------------------------------
while toc < maxTime
    
    % Read and store sensor data
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    bump               = dataStore.bump(end,[2 3 7] );
    truePose           = dataStore.truthPose(end,[2 3 4])';
    distance           = dataStore.odometry(end,2);
    angle              = dataStore.odometry(end,3);
    
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
            GPS                = wrapAroundCorrection(truePose+sqrt(0.01)*randn(size(truePose)));
            dataStore.GPS      = [dataStore.GPS; [toc GPS']];
            hFunc              = 'hGPS';
            hJacFunc           = 'HjacGPS';
            hParams            = struct();
            [mu,sigma]         = EKF(dataStore.ekfMu(end,2:4)',reshape(dataStore.ekfSigma(end,2:end),3,3),...
                gFunc,gJacFunc,Ut,hFunc,hJacFunc,hParams,GPS,R,Q,Inf);
            %=========================================================
        end
        if strcmp(filter,'ekfdepth')
            %=========================================================
            %   EKF WITH DEPTH
            %=========================================================
            hFunc        = 'depthPredict';
            hJacFunc     = 'HjacDepth';
            time         = dataStore.odometry(end,1)- dataStore.odometry(end-1,1);
            delay        = dataStore.rsdepth(end,2);
            correction   = (Ut./time).*delay;
            hParams   = struct('map',map,'sensorOrigin',sensorOrigin,'angles',angles,'correction',correction);
            [mu,sigma] = EKF(dataStore.ekfMu(end,2:4)',reshape(dataStore.ekfSigma(end,2:end),3,3),...
                gFunc,gJacFunc,Ut,hFunc,hJacFunc,hParams,dataStore.rsdepth(end,3:end)',R,Q,0.5);
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
        hParams  = struct('map',map,'sensorOrigin',sensorOrigin,'angles',angles);
        % Call particle filter function
        [particleSet]       = PF(particleSet,gFunc,hFunc,hParams,Ut,dataStore.rsdepth(end,3:end)',map);
        dataStore.particles = [dataStore.particles; toc reshape(particleSet,1,3*setSize)];
        %=============================================================
    end
    
    %  MOVE ROBOT
    %-------------
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        % Set forward and angular velocity
        cmdV = 0.4;
        cmdW = -0.15;
        if sum(bump)>0 % Sum>0 if atleast one of BumpRight, BumpLeft, BumpFront = 1
            SetFwdVelAngVelCreate(CreatePort, 0,0 );    % Stop moving forward
            pause(0.05);
            travelDist(CreatePort, cmdV, -0.25);
            pause(0.05);
            turnAngle(CreatePort, -cmdW, -15);
            pause(0.05);
        else
            SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
            pause(0.15)
        end
    end
end
%--------------------------------------------------------------------------
% PLOT GRAPHS
%--------------------------------------------------------------------------
if strcmp(filter,'ekfgps')
    plotEKFTrajectories(map,'GPS');
end
if strcmp(filter,'ekfdepth')
    plotEKFTrajectories(map,'Depth');
end
if strcmp(filter,'pf') 
    plotParticles(map,setSize);
end

end

