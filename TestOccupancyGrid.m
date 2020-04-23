function TestOccupancyGrid(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
% TESTOCCUPANCYGRID 
% Test function for MAE 4180/5180 CS 3758, Homework 5. 
% Plots the final occupancy grids using the bump sensor and using sonar.
% Will create two (2) figures containing the occupany grids.
%
%       TestOccupancyGrid(dataStore,ell_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
%
%       INPUTS:
%           dataStore   struct from running SimulatorGUI
%           l_0         initial log odds
%           NumCellsX   number of cells in the x direction (integer)
%           NumCellsY   number of cells in the y direction (integer)
%           boundaryX   boundary of the environment in the x direction.
%                       1 x 2 array [min_x max_x]
%           boundaryY   boundary of the environment in the Y direction.
%                       1 x 2 array [min_y max_y]
%       OUTPUTS:
%           none
%       Figures Created:
%           Figure 1    Occupancy grid using bump sensor
%           Figure 2    Occupancy grid from depth information
%
% Autonomous Mobile Robots
% 
gridResolution = [(boundaryX(2)-boundaryX(1))/NumCellsX...
                  (boundaryY(2)-boundaryY(1))/NumCellsY];
              
x = linspace(boundaryX(1),boundaryX(2),NumCellsX+1);
x = x(1,1:end-1)+ gridResolution(1)*ones(1,NumCellsX)/2; %X co-ordinates of grid centers
y = linspace(boundaryY(1),boundaryY(2),NumCellsY+1);
y = y(1,1:end-1)+ gridResolution(2)*ones(1,NumCellsY)/2;

logOdds_bump = logOddsBump(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY);
%save('logOdds_bump_50.mat','logOdds_bump');
logOdds_bump(logOdds_bump>=0)=1;
logOdds_bump(logOdds_bump<0)=0;
plotOccupancyGrid(x,y,'bump',reshape(logOdds_bump(end,:),NumCellsY,NumCellsX));

logOdds_depth = logOddsDepth(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY);
%save('logOdds_depth_50.mat','logOdds_depth');
logOdds_depth(logOdds_depth>=0)=1;
logOdds_depth(logOdds_depth<0)=0;
plotOccupancyGrid(x,y,'depth',reshape(logOdds_depth(end,:),NumCellsY,NumCellsX));

