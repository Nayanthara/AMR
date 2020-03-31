function[grid,gridResolution] = createGrid(gridSize,bound)
% Create array of grid point coordinates given grid size, and bound
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #3
%   SAJAN, NAYANTHARA
 
gridResolution = [(bound(2)-bound(1))/gridSize(1)...
                  (bound(4)-bound(3))/gridSize(2)];
x = bound(1)+gridResolution(1)/2:gridResolution(1):bound(2)-gridResolution(1)/2;
y = ones(1,gridSize(1))*gridResolution(2)/2;
grid =[x' y'];
for i = 1:gridSize(2)-1
    grid = [grid; x' y'+i*gridResolution(2)];
end

              
             