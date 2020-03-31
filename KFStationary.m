function[mu,cov] = KFStationary(map, measurements,mu0,cov0)
% KFSTATIONARY: Function for localization using Kalman Filter
% given a map, range measurements and initial position
%
%   [MU,COV] = KFSTATIONARY(MAP,MEASUREMENTS,GRIDSIZE,MU0) returns
%   the estimated mean and covariance for the position of the robot.  
% 
%   INPUTS
%       map             The name of the map .mat file (e.g. 'HW3map.mat')
%       measurements	The name of the measurement .txt file (e.g. 'stationary.txt')
%       mu0             The initial position for Stationary KF 2x1
%       cov0            The initial covariance matrix for Stationary KF 2x2
%
%   OUTPUT
%       mu              Mean of estimated position
%       cov             Covariance matrix of estimated position
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #3
%   SAJAN, NAYANTHARA

% Initialize mu, cov, Q and C
mu = mu0;
cov = cov0;
Q = diag([0.1 0.3 0.1 0.3]);
C = [0 -1;-1 0;0 1;1 0];

% Kalman Filter Implementation
for t = 1:size(measurements,1)
    
    predictedRange = arrayfun(@(x) rangePredict([mu' x],map,[0 0],[0]),[pi/2;0;-pi/2; pi]);
    nonNaNIndices = find(~isnan(measurements(t,:)'));
    
    % Resize matrices to exclude NaN
    Qt = Q(nonNaNIndices,nonNaNIndices);
    Ct = C(nonNaNIndices,:);
    
    % Update step
    error = measurements(t,nonNaNIndices)'-predictedRange(nonNaNIndices);
    K = cov*Ct'*inv(Ct*cov*Ct'+Qt);
    mu = mu+K*(error);
    cov = (eye(2)-K*Ct)*cov;
    
end

end