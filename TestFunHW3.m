function TestFunHW3(map, measurements, gridSize, mu0)
% Test function for Homework 3.  
% This function checks the student's grid localization and stationary KF implementation.
% 
%   INPUTS
%       map             The name of the map .mat file (e.g. 'HW3map.mat')
%       measurements	The name of the measurement .txt file (e.g. 'stationary.txt')
%       gridSize        Number of cells in the X dimension x number of cells in the Y dimension 	1x2 [n m]
%       mu0             The initial position for Stationary KF 2x1
%
%   OUTPUTS
%       The function generates plots as detailed below
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #3
%   SAJAN, NAYANTHARA

% Load map
mapStruct = load(map);
mapFields = fields(mapStruct);
map = mapStruct.(mapFields{1});

% Load measurements
measurements = load(string(measurements));

%~~~~~~~~~~~~~~~~~~~~~
% Grid localization
%~~~~~~~~~~~~~~~~~~~~~

% Find the outer walls
outerBound = [min(map(:,[1 3]),[],'all') max(map(:,[1 3]),[],'all')...
              min(map(:,[2 4]),[],'all') max(map(:,[2 4]),[],'all')];
          
% Get grid coordinates
[grid,gridResolution] = createGrid(gridSize,outerBound);

% Initialize pdf as a uniform distribution
pdf = ones(gridSize(2),gridSize(1))/prod(gridSize);

% Plot initial pdf
figure
title("Grid Localization Initial Belief Distribution");
xlabel("X Coordinate");
ylabel("Y Coordinate");
hold on
plotGridBelief(grid(:,1), grid(:,2), pdf);
for i = 1:size(map,1)
hold on
plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'-b','LineWidth',2);
end
hold off

% Call gridLocalizationStationary
[pdf] = gridLocalizationStationary(map, measurements, grid, gridResolution, pdf)

% Plot final pdf
figure
title("Grid Localization Updated Belief Distribution");
xlabel("X Coordinate");
ylabel("Y Coordinate");
hold on
plotGridBelief(grid(:,1), grid(:,2), pdf);
for i = 1:size(map,1)
    hold on
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'-b','LineWidth',2);
end
hold off

%~~~~~~~~~~~~~~~~~~~~~
% Stationary KF
%~~~~~~~~~~~~~~~~~~~~~

% Call KFStationary
cov0 = [10 0;0 10];
[mu,cov] = KFStationary(map, measurements,mu0,cov0);

% Plot initial pdf
figure
title("Kalman Filter Initial Belief Distribution");
xlabel("X Coordinate");
ylabel("Y Coordinate");
hold on
scatter(mu0(1),mu0(2),'*r')
hold on
plotCovEllipse(mu0,cov0,[1],[{'color'},{'r'}],gcf);
for i = 1:size(map,1)
    hold on
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'-b','LineWidth',2);
end
hold off

% Plot final pdf
figure
title("Kalman Filter Updated Belief Distribution");
xlabel("X Coordinate");
ylabel("Y Coordinate");
hold on
scatter(mu(1),mu(2),'*r')
hold on
plotCovEllipse(mu,cov,[1],[{'color'},{'r'}],gcf);
for i = 1:size(map,1)
    hold on
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'-b','LineWidth',2);
end
hold off

end
% END
