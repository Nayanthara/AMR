function[pdf] = gridLocalizationStationary(map, measurements, grid, gridResolution, pdf0)
% GRIDLOCALIZATIONSTATIONARY: Function to compute the grid belief (pdf)
% given a map, range measurements and grid size
%
%   PDF = GRIDLOCALIZATIONSTATIONARY(MAP,MEASUREMENTS,GRIDSIZE) returns
%   the probablity density over the grid for the position of the robot.  
% 
%   INPUTS
%       map             The name of the map .mat file (e.g. 'HW3map.mat')
%       measurements	The name of the measurement .txt file (e.g. 'stationary.txt')
%       grid            Nx2 Matrix of grid coordinates [x y]
%       gridResolution  1x2 Resolution of grid in [x y]
%       pdf0            NxM Initial pdf
%
%   OUTPUT
%       pdf             Matrix of grid belief
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #3
%   SAJAN, NAYANTHARA

% Initialize pdf as a uniform distribution
pdf = pdf0;

% Grid coordinates to pdf indices
% (grid(i,:)+grid(1,:))./gridResolution

% Pdf indices to grid coordinates
% ([x y].*gridResolution)-grid(1,:)

% Measurement model
sigma = sqrt([0.1 0.3 0.1 0.3]);

% Compute predicted measurements
predictedRange =[];
for i = 1:size(grid,1)
    predictedRange = [predictedRange; arrayfun(@(x) rangePredict([grid(i,:) x],map,[0 0],[0]),[pi/2 0 -pi/2 pi])];
end

% Calculate pdf
for t = 1:size(measurements,1)
    for i = 1:size(grid,1)
        index = int8((grid(i,:)+grid(1,:))./gridResolution);
        pd_i = normpdf(measurements(t,:),predictedRange(i,:),sigma);
        zeta = 10^sum(~isnan(pd_i));
        pdf(index(2),index(1))= zeta*prod(pd_i,'all','omitnan')*pdf(index(2),index(1));
    end
    % Normalize pdf
    pdf = pdf/sum(pdf,'all');
end

end


