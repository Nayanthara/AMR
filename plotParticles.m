function[] = plotParticles(map,psize)

global dataStore;

figure
hold on
initSet = reshape(dataStore.particles(1,2:end),psize,3);
x = initSet(:,1);
y = initSet(:,2);
u = cos(initSet(:,3));
v = sin(initSet(:,3));
quiver(x,y,u,v,0.25)

hold on
finSet = reshape(dataStore.particles(end,2:end),psize,3);
x = finSet(:,1);
y = finSet(:,2);
scatter(x,y,'or');

for i = 1:size(map,1)
    hold on
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'-b','LineWidth',2);
end
hold off
title(['Localization using Particle Filter ( '+string(psize)+' particles)']);
xlabel("X Coordinate");
ylabel("Y Coordinate");
legend('Initial particle set','Final particle set');

%==========================================================================
figure
hold on
for i=8-(1:6)
    x = dataStore.particles(:,i);
    y = dataStore.particles(:,psize+i);
    plot(x,y,'LineWidth',1.2);
end
plot(dataStore.truthPose(:,2)',dataStore.truthPose(:,3)','-.','LineWidth',1.5);
hold on
for i = 1:size(map,1)
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'-b','LineWidth',2);
end

hold off
title(['Localization using Particle Filter ( '+string(psize)+' particles)']);
xlabel("X Coordinate");
ylabel("Y Coordinate");
legend('6th','5th','4th','3rd','2nd','Best Estimate','True pose');
%==========================================================================
figure
hold on
x = sum(dataStore.particles(:,2:11),2)./10;
y = sum(dataStore.particles(:,psize+2:psize+11),2)./10;
plot(x,y,'LineWidth',1.2);
plot(dataStore.truthPose(:,2)',dataStore.truthPose(:,3)','-.','LineWidth',1.5);
for i = 1:size(map,1)
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'-b','LineWidth',2);
end
hold off
title(['Localization using Particle Filter ( '+string(psize)+' particles)']);
xlabel("X Coordinate");
ylabel("Y Coordinate");
legend('Average of best 10','True pose');
end