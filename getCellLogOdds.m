function[cellLogOdds] = getCellLogOdds(pose,bump,boundaryX,boundaryY,gridResolution)
%This function computes the logOdds in the local region of the robot at one
%time step.

%Output
%cellLogOdds  Nx[rowNumber columnNumber logOdds]

robotRadius = 0.17;
discreteRadius = ceil(0.17./gridResolution);

%convert pose to grid indices
rj = ceil((pose(1)-boundaryX(1))/gridResolution(1));
ri = ceil((pose(2)-boundaryY(1))/gridResolution(2));

cellLogOdds = [];
for i= ri-discreteRadius(2):ri+discreteRadius(2)
    for j= rj-discreteRadius(1):rj+discreteRadius(1)
        if(i>0&&j>0)
            cellLogOdds = [cellLogOdds;i j -7];
        end
    end
end
d = robotRadius+norm(gridResolution);
jvals = ceil((( pose(1)-boundaryX(1)+d.*[cos(pose(3)+pi/6) cos(pose(3)) cos(pose(3)-pi/6)]))/gridResolution(1));
ivals = ceil(((-pose(2)+boundaryY(2)-d.*[sin(pose(3)+pi/6) sin(pose(3)) sin(pose(3)-pi/6)]))/gridResolution(2));
for i = 1:3
    if(bump(i)>0)
        cellLogOdds = [cellLogOdds; ivals(i) jvals(i) 9];
    end
end
