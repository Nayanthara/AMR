function[dataStore] = backupBump(Robot,maxTime)
% BACKUPBUMP: program to drive the robot forward at constant velocity until it bumps into
% something. If a bump sensor is triggered, command the robot to back up 0.25m and turn
% clockwise 30 degrees before continuing to drive forward again. program saves datalog.
% 
%   dataStore = BACKUPBUMP(Robot,maxTime) runs 
% 
%   INPUTS type
%       CreatePort  Create port object (get from running RoombaInit)
%       DistPort    Depth ports object (get from running CreatePiInit)
%       TagPort     Tag ports object (get from running CreatePiInit)
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   SAJAN, NAYANTHARA

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end


% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

tic
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    % Extact relevant fields into another array
    bump = dataStore.bump(end,[2 3 7] );
    
    % CONTROL FUNCTION (send robot commands)
    
    % Set forward velocity
    [cmdV,cmdW]= limitCmds(0.45,0,0.1,0.13);
    SetFwdVelAngVelCreate(CreatePort, cmdV,cmdW);
    
    if sum(bump)>0   % Sum>0 if atleast one of BumpRight, BumpLeft, BumpFront = 1
        SetFwdVelAngVelCreate(CreatePort, 0,0 );    % Stop moving forward
        pause(0.1);
        travelDist(CreatePort, 0.45, -0.25);    % Move 0.25m backward at 0.45m/s
        turnAngle(CreatePort, 0.15, -30);   % Turn 30 degrees clockwise at 0.15rad/s
        SetFwdVelAngVelCreate(CreatePort, cmdV,cmdW);   
    else
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
        
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
    
    pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );

