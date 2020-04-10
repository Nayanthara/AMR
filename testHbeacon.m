dataStore = load('EKFGPSRun2.mat').dataStore;
beaconLoc = load('labBeacon.mat').beaconLoc;
sensorOrigin = [0.13 0];
mapStruct               = load('labmap.mat');
mapFields               = fields(mapStruct);
map                     = mapStruct.(mapFields{1});
beacon = [];
i=2;
while i <= size(dataStore.truthPose,1)
    hParams      = struct('map',map,'sensorOrigin',sensorOrigin,'beaconLoc',beaconLoc);
    tags         = hBeacon(dataStore.truthPose(i,2:end)',hParams)
        if ~isempty(tags)
            beacon = [beacon ; tags];
        end
    i = i+1;
end
beacon
figure
axis equal
for i = 1:size(map,1)
    hold on
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'-b','LineWidth',2);
end
plot(dataStore.truthPose(:,2)',dataStore.truthPose(:,3)','-.','LineWidth',1.5);
hold on
x = beaconLoc(:,2);
y = beaconLoc(:,3);
scatter(x,y,'*r');
dx = 0.1; dy = 0.1; % displacement so the text does not overlay the data points
text(x+dx, y+dy, num2str(beaconLoc(:,1)));