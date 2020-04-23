function[] = plotTrajectories(map_name,dataStore,txt,boundaryX,boundaryY,NumCellsX,NumCellsY)
% This function plots the trajectory and bump sensed

mapStruct               = load(map_name);
mapFields               = fields(mapStruct);
map                     = mapStruct.(mapFields{1});

figure
axis equal
hold on
p1 = plot(dataStore.truthPose(:,2)',dataStore.truthPose(:,3)','-.','LineWidth',1.2);
for x1 = linspace(boundaryX(1),boundaryX(2),NumCellsX+1)
    plot([x1 x1],boundaryY,'Color', [168 168 168]/255);
end
for y1 =linspace(boundaryY(1),boundaryY(2),NumCellsY+1)
    plot(boundaryX,[y1 y1],'Color', [168 168 168]/255);
end
for i = 1:size(map,1)
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'-b','LineWidth',2);
end
bump = sum(dataStore.bump(:,[2 3 7]),2);
x = dataStore.truthPose(find(bump),2);
y = dataStore.truthPose(find(bump),3);
p2 = scatter(x,y,'*r');
title(['Mapping using '+string(txt)+' sensor measurements']);
xlabel("X Coordinate");
ylabel("Y Coordinate");
legend([p1 p2],'True trajectory','Bump sensed');
hold off

end
