%% References
%
% [1] Lang, Alex H., et al. "PointPillars: Fast encoders for object
% detection from point clouds." Proceedings of the IEEE Conference on
% Computer Vision and Pattern Recognition. 2019.
%
% [2] Zhou, Yin, and Oncel Tuzel. "Voxelnet: End-to-end learning for point
% cloud based 3d object detection." Proceedings of the IEEE Conference on
% Computer Vision and Pattern Recognition. 2018.
% 
% [3] Yang, Bin, Wenjie Luo, and Raquel Urtasun. "Pixor: Real-time 3d
% object detection from point clouds." Proceedings of the IEEE conference
% on Computer Vision and Pattern Recognition. 2018.

%% Track-Level Fusion of Lidar Data shared by Two Neighbor Vehicles

%% Setup Scenario for Synthetic Data Generation
% For reproducible results
clear
rng(2021);

% Create Scenario 
[scenario, egoVehicle, radars, lidarEgo1] = helperCreateRadarLidarScenario;
[scenario2, egoVehicle2, radars2, lidarEgo2] = helperCreateRadarLidarScenario;
% [allData, scenario, egoVehicle, lidar] = createIntersectionScenario;

%% Radar Tracking Algorithm
radarTrackingAlgorithm = helperRadarTrackingAlgorithm(radars);

%% Lidar Tracking Algorithm for Ego 1
lidarTrackingAlgorithmEgo1 = helperLidarTrackingAlgorithm(lidarEgo1);
lidarTrackingAlgorithmEgo2 = helperLidarTrackingAlgorithm(lidarEgo2);

%% Set Up Fuser, Metrics, and Visualization
radarConfig = fuserSourceConfiguration('SourceIndex',1,...
    'IsInitializingCentralTracks',true,...
    'CentralToLocalTransformFcn',@central2radar,...
    'LocalToCentralTransformFcn',@radar2central);

lidarConfig = fuserSourceConfiguration('SourceIndex',2,...
    'IsInitializingCentralTracks',true);

fEgo1 = lidarTrackingAlgorithmEgo1.StateTransitionFcn;
fEgo2 = lidarTrackingAlgorithmEgo2.StateTransitionFcn;

% Create a trackFuser object
fuser = trackFuser('SourceConfigurations',{radarConfig;lidarConfig},...
    'StateTransitionFcn',fEgo1,...
    'ProcessNoise',diag([1 3 1]),...
    'HasAdditiveProcessNoise',false,...
    'AssignmentThreshold',[250 inf],...
    'ConfirmationThreshold',[3 5],...
    'DeletionThreshold',[5 5],...
    'StateFusion','Custom',...
    'CustomStateFusionFcn',@helperRadarLidarFusionFcn);

% Radar GOSPA
gospaRadar = trackGOSPAMetric('Distance','custom',...
    'DistanceFcn',@helperRadarDistance,...
    'CutoffDistance',25);

% Lidar GOSPA
gospaLidar = trackGOSPAMetric('Distance','custom',...
    'DistanceFcn',@helperLidarDistance,...
    'CutoffDistance',25);

% Central/Fused GOSPA
gospaCentral = trackGOSPAMetric('Distance','custom',...
    'DistanceFcn',@helperLidarDistance,...% State-space is same as lidar
    'CutoffDistance',25);

display = helperMultiLidarTrackFusionDisplay('FollowActorID',3);
showLegend(display,scenario);

% Initialzie GOSPA metric and its components for all tracking algorithms.
gospa = zeros(3,0);
missTarget = zeros(3,0);
falseTracks = zeros(3,0);

% Initialize fusedTracks
fusedTracks = objectTrack.empty(0,1);

% A counter for time steps elapsed for storing gospa metrics.
idx = 1;

% Ground truth for metrics. This variable updates every time-step
% automatically being a handle to the actors.
groundTruth = scenario.Actors(2:end);
groundTruth2 = scenario2.Actors(2:end);

% RUN SCENARIO
while advance(scenario)
    % Current time
    time = scenario.SimulationTime;
    %display(scenario)
    % Collect radar and lidar measurements and ego pose to track in
    % scenario frame. See helperCollectSensorData below.
    [radarDetections, ptCloud1, ptCloud2, egoPose] = helperCollectSensorData(egoVehicle, radars, lidarEgo1,lidarEgo2, time);
    
    % Generate radar tracks
    radarTracks = radarTrackingAlgorithm(egoPose, radarDetections, time);
        
    % Generate lidar tracks and analysis information like bounding box
    % detections and point cloud segmentation information
    [lidarTracks, lidarDetections, segmentationInfo] = ...
        lidarTrackingAlgorithmEgo1(egoPose, ptCloud1, time);
    
    % Concatenate radar and lidar tracks
    localTracks = [radarTracks;lidarTracks];
    
    % Update the fuser. First call must contain one local track
    if ~(isempty(localTracks) && ~isLocked(fuser))
        fusedTracks = fuser(localTracks,time);
    end
    
    % Capture GOSPA and its components for all trackers 
    % Generalized optimal subpattern assignment (GOSPA) metric
    [gospa(1,idx),~,~,~,missTarget(1,idx),falseTracks(1,idx)] = gospaRadar(radarTracks, groundTruth);
    [gospa(2,idx),~,~,~,missTarget(2,idx),falseTracks(2,idx)] = gospaLidar(lidarTracks, groundTruth);
    [gospa(3,idx),~,~,~,missTarget(3,idx),falseTracks(3,idx)] = gospaCentral(fusedTracks, groundTruth);
    
    % Update the display
    %display(scenario, radars);
    display(scenario, radars, radarDetections, radarTracks, ...
        lidarEgo1, ptCloud1, lidarDetections, segmentationInfo, lidarTracks,...
        fusedTracks);
    
    % Update the index for storing GOSPA metrics
    idx = idx + 1;
end

% Update example animations
updateExampleAnimations(display);

%% Evaluate Performance
% Plot missed target component
figure; plot(missTarget','LineWidth',2); legend('Radar','Lidar','Fused');
title("Missed Target Metric"); xlabel('Time step'); ylabel('Metric'); grid on;

% Plot false track component
figure; plot(falseTracks','LineWidth',2); legend('Radar','Lidar','Fused');
title("False Track Metric"); xlabel('Time step'); ylabel('Metric'); grid on;

% Plot GOSPA
figure; plot(gospa','LineWidth',2); legend('Radar','Lidar','Fused');
title("GOSPA Metric"); xlabel('Time step'); ylabel('Metric'); grid on;