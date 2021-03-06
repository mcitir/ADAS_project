function [sensors, numSensors, attachedVehicle] = createSensors(scenario)
% createSensors Returns all sensor objects to generate detections
% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);

% Vehicle 1
sensors{1} = lidarPointCloudGenerator('SensorIndex', 1, ...
    'SensorLocation', [0.95 0], ...
    'MaxRange', 150, ...
    'HasNoise', false, ...
    'ActorProfiles', profiles);

% Vehicle 2
sensors{2} = lidarPointCloudGenerator('SensorIndex', 2, ...
    'SensorLocation', [0.95 0], ...
    'MaxRange', 150, ...
    'HasNoise', false, ...
    'ActorProfiles', profiles);
attachedVehicle = [1;2];
numSensors = numel(sensors);


% % Vehicle 1 radar reports clustered detections
% sensors{1} = radarDataGenerator('No scanning', 'SensorIndex', 1, 'UpdateRate', 10, ...
%     'MountingLocation', [3.7 0 0.2], 'RangeLimits', [0 50], 'FieldOfView', [60 5], ...
%     'RangeResolution', 2.5, 'AzimuthResolution', 4, ...
%     'Profiles', profiles, 'HasOcclusion', true, 'HasFalseAlarms', false, ...
%     'TargetReportFormat', 'Clustered detections');
% 
% % Vehicle 2 radar reports tracks
% sensors{2} = radarDataGenerator('No scanning', 'SensorIndex', 2, 'UpdateRate', 10, ...
%     'MountingLocation', [3.7 0 0.2], 'RangeLimits', [0 120], 'FieldOfView', [30 5], ...
%     'RangeResolution', 2.5, 'AzimuthResolution', 4, ...
%     'Profiles', profiles, 'HasOcclusion', true, 'HasFalseAlarms', false, ...
%     'TargetReportFormat', 'Tracks', 'DeletionThreshold', [3 3]);
% 
% % Vehicle 1 vision sensor reports detections
% sensors{3} = visionDetectionGenerator('SensorIndex', 3, ...
%     'MaxRange', 100, 'SensorLocation', [1.9 0], 'DetectorOutput', 'Objects only', ...
%     'ActorProfiles', profiles);
% attachedVehicle = [1;2;1];
% numSensors = numel(sensors);
end