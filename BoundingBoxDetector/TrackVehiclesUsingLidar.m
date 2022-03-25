%% References
% Arya Senna Abdul Rachman, Arya. "3D-LIDAR Multi Object Tracking for
% Autonomous Driving: Multi-target Detection and Tracking under Urban Road
% Uncertainties." (2017).

clear;clc;
load("10_sec_multivehicle.mat")
%% Track Vehicles Using Lidar: From Point Cloud to Track List
% Load data if unavailable. The lidar data is stored as a cell array of
% pointCloud objects.
% if ~exist('lidarData','var')
%     dataURL = 'https://ssd.mathworks.com/supportfiles/lidar/data/TrackVehiclesUsingLidarExampleData.zip';
%     datasetFolder = fullfile(tempdir,'LidarExampleDataset');
%     if ~exist(datasetFolder,'dir')
%        unzip(dataURL,datasetFolder);
%     end
%     % Specify initial and final time for simulation.
%     initTime = 0;
%     finalTime = 35;
%     [lidarData, imageData] = loadLidarAndImageData(datasetFolder,initTime,finalTime); %function
% end
% lidarData_from_matlab_source_link = lidarData;

%imageData= uint8(1);

datasetFolder = fullfile('LidarExampleDataset');
initTime = 0;
finalTime = 35;
[lidarData_from_matlab_source_link, imageData] = loadLidarAndImageData(datasetFolder,initTime,finalTime); %function

%% Conversion of Point Clouds to lidarData Format
file_name = ego_blue_10sec;
lidarData_new{size(file_name,2),1} = []; 
for i=1:size(file_name,2)
    lidarData_new{i,1} = file_name(i).PointClouds{1,1};
end

% structured to unstructured
lidarData = lidarData_new;

% To remove NaN, Inf point clouds, 
% set argument removingInvalidPoints as true| default is false
% FUNCTION unstructuredLidarDataSet = structured2unstructuredPointCloud(lidarDataSet, removingInvalidPoints)

lidarData = structured2unstructuredPointCloud(lidarData,true); 


%% 
% Set random seed to generate reproducible results.
S = rng(2018);

% A bounding box detector model.
detectorModel = HelperBoundingBoxDetector(... %function
    'XLimits',[-50 75],...              % min-max
    'YLimits',[-5 5],...                % min-max
    'ZLimits',[-2 5],...                % min-max
    'SegmentationMinDistance',1.8,...   % minimum Euclidian distance
    'MinDetectionsPerCluster',1,...     % minimum points per cluster
    'MeasurementNoise',blkdiag(0.25*eye(3),25,eye(3)),...       % measurement noise in detection report
    'GroundMaxDistance',0.3);           % maximum distance of ground points from ground plane

%%
% Set the |HasDetectableTrackIDsInput| property of the tracker as |true|,
% which enables you to specify a state-dependent probability of detection.
% The detection probability of a track is calculated by the
% |helperCalcDetectability| function, listed at the end of this example.

assignmentGate = [75 1000]; % Assignment threshold;
confThreshold = [7 10];    % Confirmation threshold for history logic
delThreshold = [8 10];     % Deletion threshold for history logic
Kc = 1e-9;                 % False-alarm rate per unit volume

% IMM filter initialization function
filterInitFcn = @helperInitIMMFilter; %function

% A joint probabilistic data association tracker with IMM filter
tracker = trackerJPDA('FilterInitializationFcn',filterInitFcn,...
    'TrackLogic','History',...
    'AssignmentThreshold',assignmentGate,...
    'ClutterDensity',Kc,...
    'ConfirmationThreshold',confThreshold,...
    'DeletionThreshold',delThreshold,...
    'HasDetectableTrackIDsInput',true,...
    'InitializationThreshold',0,...
    'HitMissThreshold',0.1);

%% Create display
displayObject = HelperLidarExampleDisplay([],... %imageData{1}, reference image deleted
    'PositionIndex',[1 3 6],...
    'VelocityIndex',[2 4 7],...
    'DimensionIndex',[9 10 11],...
    'YawIndex',8,...
    'MovieName','',...  % Specify a movie name to record a movie.
    'RecordGIF',false); % Specify true to record new GIFs

%% Loop Through Data
% Loop through the recorded lidar data, generate detections from the
% current point cloud using the detector model and then process the
% detections using the tracker.
time = 0;       % Start time
dT = 0.1;       % Time step

% Initiate all tracks.
allTracks = struct([]);

% Initiate variables for comparing MATLAB and MEX simulation.
numTracks = zeros(numel(lidarData),2);

% Loop through the data
for i = 1:numel(lidarData)
    % Update time
    time = time + dT;
    
    % Get current lidar scan
    currentLidar = lidarData{i};
    
    % Generator detections from lidar scan.
    [detections,obstacleIndices,groundIndices,croppedIndices] = detectorModel(currentLidar,time);
    
    % Calculate detectability of each track.
    detectableTracksInput = helperCalcDetectability(allTracks,[1 3 6]);
    
    % Pass detections to track.
    [confirmedTracks,tentativeTracks,allTracks,info] = tracker(detections,time,detectableTracksInput);
    numTracks(i,1) = numel(confirmedTracks);

    % Get model probabilities from IMM filter of each track using
    % getTrackFilterProperties function of the tracker.
    modelProbs = zeros(2,numel(confirmedTracks));
    stateTest= [];
    for k = 1:numel(confirmedTracks)
        c1 = getTrackFilterProperties(tracker,confirmedTracks(k).TrackID,'ModelProbabilities');
        modelProbs(:,k) = c1{1};
        stateTest = cat(2,stateTest,confirmedTracks(k).State);
    end
    
    % Update display
    if isvalid(displayObject.PointCloudProcessingDisplay.ObstaclePlotter)
        % Get current image scan for reference image
        currentImage = imageData{i};
        
        % Update display object
        displayObject(detections,confirmedTracks,currentLidar,obstacleIndices,...
            groundIndices,croppedIndices,currentImage,modelProbs);
    end
    
    % Snap a figure at time = 18
    if abs(time - 18) < dT/2
        snapnow(displayObject);
    end
end

% Write movie if requested
if ~isempty(displayObject.MovieName)
    writeMovie(displayObject);
end

% Write new GIFs if requested.
if displayObject.RecordGIF
    % second input is start frame, third input is end frame and last input
    % is a character vector specifying the panel to record.
    writeAnimatedGIF(displayObject,10,170,'trackMaintenance','ego');
    writeAnimatedGIF(displayObject,310,330,'jpda','processing');
    writeAnimatedGIF(displayObject,120,140,'imm','details');
end

% figure;
% for si=1:size(lidarData,1)
%     %pcshow(lidarData{si,1}.Location)
%     pcshowpair(ego_red_10sec(si).PointClouds{1,1},...
%         ego_blue_10sec(si).PointClouds{1,1})
%     pause(0.05)
% end