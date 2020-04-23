function plotOccupancyGrid(coodinatesXOfGrid,coodinatesYOfGrid,txt,logOdds)
% plotOccupancyGrid
%                          
%   INPUTS
%       coodinatesXOfGrid   x coorindates of grid centers (1xm)
%       coodinatesYOfGrid   y coorindates of grid centers (1xn)
%       logOdds               (mxn) matrix of logOdds


% plot occupancy grid
figure
axis equal
image(coodinatesXOfGrid,coodinatesYOfGrid,logOdds,'CDataMapping','scaled');
colormap (flipud(gray));
colorbar
caxis([min(min(logOdds)) max(max(logOdds))]);
title(['Occupancy grid using '+string(txt)+' sensor measurements']);
% flip axis (to make y point up)
ax = gca;
ax.YDir = 'normal';
