function[logOdds] = logOddsDepth(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
% This function calculates the logOdds iteratively


% Grid coordinates to indices
% (grid(i,:)+grid(1,:))./gridResolution

% Indices to grid coordinates
% ([x y].*gridResolution)-grid(1,:)

% Calculate grid resolution
gridResolution = [(boundaryX(2)-boundaryX(1))/NumCellsX...
                  (boundaryY(2)-boundaryY(1))/NumCellsY];
              
%Create and initialize occupancy grid
logOdds_0 = l_0*ones(NumCellsY,NumCellsX);
logOdds = reshape(logOdds_0,1,NumCellsY*NumCellsX);

x = linspace(boundaryX(1),boundaryX(2),NumCellsX+1);
y = linspace(boundaryY(1),boundaryY(2),NumCellsY+1);
[R,C]=meshgrid(x,y);
x = x(1,1:end-1)+ gridResolution(1)*ones(1,NumCellsX)/2; 
y = y(1,1:end-1)+ gridResolution(2)*ones(1,NumCellsY)/2;

angles = linspace(deg2rad(27),-deg2rad(27),9);
i = 1;
k = 1;
while i <= size(dataStore.truthPose,1)
    if(dataStore.truthPose(i,1)>dataStore.rsdepth(k,1))
        k = k+1;
        continue;
    end
    logOdds_t = zeros(NumCellsY,NumCellsX); % Initialize for this timestep;
    validIndices = find(~isnan(dataStore.rsdepth(k,3:11))& dataStore.rsdepth(k,3:11)> 0.15);
    obsX = dataStore.truthPose(i,2)+dataStore.rsdepth(k,3:11).*cos(angles+dataStore.truthPose(i,4));
    obsY = dataStore.truthPose(i,3)+dataStore.rsdepth(k,3:11).*sin(angles+dataStore.truthPose(i,4));
%     obsX = obsX(validIndices)
%     obsY = obsY(validIndices)
    freeCells =[];
    for j = validIndices
        if (boundaryX(1)<obsX(j)&& boundaryY(1)<obsY(j)&& boundaryX(2)>obsX(j)&& boundaryY(2)>obsY(j))
            freeCells = [freeCells; getFreeCells(dataStore.truthPose(i,2:3),[obsX(j) obsY(j)],R,C,NumCellsX,NumCellsY,boundaryX,boundaryY,gridResolution)];
        end
    end
    for q = 1:size(freeCells,1)
        logOdds_t(freeCells(q,1),freeCells(q,2))=-7;
    end
    obsJ = ceil((obsX-boundaryX(1))./gridResolution(1));
    obsI = ceil((obsY-boundaryY(1))./gridResolution(2));
    occupiedCells = [obsI' obsJ'];
    for q = 1:size(occupiedCells,1)
        if (0<occupiedCells(q,1)&& NumCellsX>=occupiedCells(q,2)&& NumCellsY>=occupiedCells(q,1)&& 0<occupiedCells(q,2))
            logOdds_t(occupiedCells(q,1),occupiedCells(q,2))=9;
        end
    end
    logOdds = [logOdds; reshape((reshape(logOdds(end,:),NumCellsY,NumCellsX)+logOdds_t-logOdds_0),1,NumCellsY*NumCellsX)];
    i=i+1;
    k=k+1;
end






