function[logOdds] = logOddsBump(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
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
x = x(1,1:end-1)+ gridResolution(1)*ones(1,NumCellsX)/2; %X co-ordinates of grid centers
y = linspace(boundaryY(1),boundaryY(2),NumCellsY+1);
y = y(1,1:end-1)+ gridResolution(2)*ones(1,NumCellsY)/2; %Y co-ordinates of grid centers
[R,C]=meshgrid(x,y);
i = 1;
k = 1;
while i <= size(dataStore.truthPose,1)
    if(dataStore.truthPose(i,1)>dataStore.bump(k,1))
        k = k+1;
        continue;
    end
    bump = dataStore.bump(end,[2 3 7] );
    logOdds_t = zeros(NumCellsY,NumCellsX); % Initialize for this timestep;
    cellLogOdds = getCellLogOdds(dataStore.truthPose(i,2:end),bump,boundaryX,boundaryY,gridResolution);
    for q = 1:size(cellLogOdds,1)
        if(0<cellLogOdds(q,1) && cellLogOdds(q,1)<NumCellsY && 0<cellLogOdds(q,2) && cellLogOdds(q,2)<NumCellsX )
            logOdds_t(cellLogOdds(q,1),cellLogOdds(q,2))= cellLogOdds(q,3);
        end
    end
    logOdds = [logOdds; reshape((reshape(logOdds(end,:),NumCellsY,NumCellsX)+logOdds_t-logOdds_0),1,NumCellsY*NumCellsX)];
    i=i+1;
    k=k+1;
end

















