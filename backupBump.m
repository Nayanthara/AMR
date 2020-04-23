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
    maxTime = 3600;
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
i = 1;
sgn = 1;
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    % Extact relevant fields into another array
    bump = dataStore.bump(end,[2 3 7] );
    % CONTROL FUNCTION (send robot commands)
    
    if(mod(i,15)==0)
        turnAngle(Robot, 0.15,-90);
        i =i+1;
        sgn = -sgn;
    end
    
    % Set forward velocity
    [cmdV,cmdW]= limitCmds(0.45,0.15,0.25,0.15);
         
    SetFwdVelAngVelCreate(Robot, cmdV,sgn*cmdW);
    
    if sum(bump)>0   % Sum>0 if atleast one of BumpRight, BumpLeft, BumpFront = 1
        SetFwdVelAngVelCreate(Robot, 0,0 );    % Stop moving forward
        pause(0.1);
        travelDist(Robot, 0.25, -0.4);    % Move 0.4m backward at 0.25m/s
        turnAngle(Robot, 0.15, sgn*5);
        SetFwdVelAngVelCreate(Robot, cmdV,sgn*cmdW);
        i =i+1;
    else
        SetFwdVelAngVelCreate(Robot, cmdV,sgn*cmdW);
    end
        
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV,sgn*cmdW);
    end
    pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0 );

save('run_data_6.mat','dataStore');

