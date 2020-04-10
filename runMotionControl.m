data = load('EKFDepthRun3.mat').dataStore;
filter = 'ekfbeacon';
motionControl_lab(filter,data);