function[] = plotEKFTrajectories(map,txt,dataStore)
% This function plots the required graphs for EKF implementation

figure
hold on
plot(dataStore.ekfMu(:,2)',dataStore.ekfMu(:,3)','LineWidth',1.2);
plot(dataStore.deadReck(:,2)',dataStore.deadReck(:,3)','LineWidth',1.2);
plot(dataStore.truthPose(:,2)',dataStore.truthPose(:,3)','-.','LineWidth',1.2);
plotCovEllipse(dataStore.ekfMu(1,2:3),reshape(dataStore.ekfSigma(1,[2 3 5 6]),2,2),[1],[{'color'},{'r'}],gcf);
plotCovEllipse(dataStore.ekfMu(end,2:3),reshape(dataStore.ekfSigma(end,[2 3 5 6]),2,2),[1],[{'color'},{'g'}],gcf);
for i = 1:size(map,1)
    hold on
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'-b','LineWidth',2);
end
title(['Localization using EKF with '+string(txt)+' Measurements']);
xlabel("X Coordinate");
ylabel("Y Coordinate");
legend('EKF Mu','Dead Reckoning', 'True Trajectory','Initial Covariance','Final Covariance');
hold off

end
