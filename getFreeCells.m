function[cells] = getFreeCells(p1,p2,R,C,NumCellsX,NumCellsY,boundaryX,boundaryY,gridResolution)
%This function computes a list of free cells along the line joining P1 and
%P2 using intersectPoint on each side of each cell.

cmin = ceil((min(p1(1),p2(1))-boundaryX(1))/gridResolution(1));
rmin = ceil((min(p1(2),p2(2))-boundaryY(1))/gridResolution(2));
cmax = ceil((max(p1(1),p2(1))-boundaryX(1))/gridResolution(1));
rmax = ceil((max(p1(2),p2(2))-boundaryY(1))/gridResolution(2));
cells = [];
for i= rmin:rmax
    for j= cmin:cmax
        [b1,~,~,~] = intersectPoint(p1(1),p1(2),p2(1),p2(2),R(i,j),C(i,j),R(i+1,j),C(i+1,j));
        [b2,~,~,~] = intersectPoint(p1(1),p1(2),p2(1),p2(2),R(i,j),C(i,j),R(i,j+1),C(i,j+1));
        [b3,~,~,~] = intersectPoint(p1(1),p1(2),p2(1),p2(2),R(i+1,j+1),C(i+1,j+1),R(i+1,j),C(i+1,j));
        [b4,~,~,~] = intersectPoint(p1(1),p1(2),p2(1),p2(2),R(i+1,j+1),C(i+1,j+1),R(i,j+1),C(i,j+1));
         if(b1||b2||b3||b4)
             cells = [cells; i j];
         end 
    end
end
cells;
% x = linspace(boundaryX(1),boundaryX(2),NumCellsX+1);
% x = x(1,1:end-1)+ gridResolution(1)*ones(1,NumCellsX)/2; %X co-ordinates of grid centers
% y = linspace(boundaryY(1),boundaryY(2),NumCellsY+1);
% y = y(1,1:end-1)+ gridResolution(2)*ones(1,NumCellsY)/2; %Y co-ordinates of grid centers
% logOdds = 0*ones(NumCellsY,NumCellsX);
% for q = 1:size(cells,1)
%     logOdds(cells(q,1),cells(q,2))=1;
% end
% figure
% hold on
% %plotOccupancyGrid(x,y,'test',logOdds)
% plot([p1(1) p2(1)],[p1(2) p2(2)],'-b');
% for x1 =linspace(boundaryX(1),boundaryX(2),NumCellsX+1)
%     plot([x1 x1],boundaryY,'Color', [168 168 168]/255);
% end
% for y1 =linspace(boundaryY(1),boundaryY(2),NumCellsY+1)
%     plot(boundaryX,[y1 y1],'Color', [168 168 168]/255);
% end
end