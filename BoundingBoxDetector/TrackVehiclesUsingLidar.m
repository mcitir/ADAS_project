%% References
% Arya Senna Abdul Rachman, Arya. "3D-LIDAR Multi Object Tracking for
% Autonomous Driving: Multi-target Detection and Tracking under Urban Road
% Uncertainties." (2017).

clear;
close all hidden;
clc;
load("10_sec_multivehicle.mat")

%% Records Actors Position
ego1_dataset = ego_blue_10sec;
% ego2_dataset = ego_red_10sec;
%%% Blue Vehicle
for tt=1:numel(ego1_dataset)
    egoBluePose(tt,:) = ego1_dataset(tt).ActorPoses(1).Position;
end
figure

swarmchart(egoBluePose(:,1),...
         egoBluePose(:,2),'filled','MarkerFaceAlpha',0.5,'MarkerEdgeAlpha',0.5);
axis([round(min(egoBluePose(:,1))-10),...
      round((max(egoBluePose(:,1))+10)),... %inf,inf]);
      round(min(egoBluePose(:,2))),...
      round((max(egoBluePose(:,2))))]);
axis equal
% %%% Red Vehicle
% for tt=1:numel(ego2_dataset)
%     egoRedPose(tt,:) = ego2_dataset(tt).ActorPoses(1).Position;
% end
% hold
% scatter(egoRedPose(:,1),...
%          egoRedPose(:,2),'MarkerEdgeColor','r');
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
file_name = ego1_dataset;
lidarData_new{size(file_name,2),1} = [];
player = pcplayer([-100 100],[-100 100],[-100 100]);
for i=1:size(file_name,2)
    lidarData_new{i,1} = file_name(i).PointClouds{1,1};
    view(player,lidarData_new{i,1})
    pause(0.1)
end




%% Coordinate Transformation of Point Cloud from Vehicle to World Coordinate

for ps=1:size(file_name,2)
    %[x y z roll pitch yaw]
pose = [file_name(ps).ActorPoses(1).Position,... % ActorPoses(ActorID=1) for blue vehicle
        deg2rad(file_name(ps).ActorPoses(1).Roll),...
        deg2rad(file_name(ps).ActorPoses(1).Pitch),...
        deg2rad(file_name(ps).ActorPoses(1).Yaw)];

euler = [pose(4) pose(5) pose(6)];
rotEgo2World = eul2rotm(euler, 'XYZ');
trEgo2World = [pose(1) pose(2) pose(3)];
tform = rigid3d(rotEgo2World, trEgo2World);
lidarData_new_transformed{ps,1} = pctransform(lidarData_new{ps,1},tform);
end
% Player for point cloud
player = pcplayer([-100 100],[-7 7],[0 5]);
player2 = pcplayer([-300 300],[-300 300],[0 5]);


for i=1:size(file_name,2)
    view(player,lidarData_new{i,1})
    view(player2,lidarData_new_transformed{i,1})
    pause(0.1)
end

%% structured to unstructured
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

% displayObject = HelperLidarExampleDisplayUpdated([],... %imageData{1}, reference image deleted
%     'PositionIndex',[1 3 6],...
%     'VelocityIndex',[2 4 7],...
%     'DimensionIndex',[9 10 11],...
%     'YawIndex',8,...
%     'MovieName','movie',...  % Specify a movie name to record a movie.
%     'RecordGIF',false); % Specify true to record new GIFs

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


% testFieldnames = fieldnames(toStruct(objectTrack));
% strTest = cellstr(testFieldnames)
% testStruct = struct('test',{},'test2',{});
% 
% colConfirmedTracks = struct;
% colTentativeTracks = struct;
% testObjStruct = toStruct(objectTrack);
% 
% unpackStruct = @(testStruct) cellfun(@(name) assignin('base',name,getfield(testStruct,name)),fieldnames(testStruct));
% unpackStruct(testObjStruct);

% Loop through the data
dispRes=[];

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
    [confirmedTracks,tentativeTracks,allTracks,info] = tracker(detections,time,detectableTracksInput); %#ok<*SAGROW> 
    numTracks(i,1) = numel(confirmedTracks);
%     if exist("confirmedTracks")
%         disp('Exist')
%         dispRes = vertcat(dispRes,1); %#ok<AGROW> 
%     else
%         disp('Not Exist')
%         dispRes = vertcat(dispRes,0); %#ok<AGROW> 
%     end
    % Convert tracks to Global Coordinate Frame
    confirmedTracksGlob = confirmedTracks;
    for numC=1:numel(confirmedTracksGlob)
        
        pose = [ego_blue_10sec(i).ActorPoses(1).Position,... % ActorPoses(ActorID=1) for blue vehicle
                ego_blue_10sec(i).ActorPoses(1).Roll,...
                ego_blue_10sec(i).ActorPoses(1).Pitch,...
                ego_blue_10sec(i).ActorPoses(1).Yaw];
        points = [confirmedTracksGlob(numC,1).State(1), ...
                  confirmedTracksGlob(numC,1).State(3), ...
                  confirmedTracksGlob(numC,1).State(6)];
        pointsGlob=local2glob(pose,points);
        confirmedTracksGlob(numC,1).State(1)=pointsGlob(1);
        confirmedTracksGlob(numC,1).State(3)=pointsGlob(2);
        confirmedTracksGlob(numC,1).State(6)=pointsGlob(3);
    end
    %cell(confirmedTracks)
    if i==1
        colConfirmedTracks =toStruct(confirmedTracks);
        colConfirmedTracksGlob =toStruct(confirmedTracksGlob);
        colTentativeTracks = toStruct(tentativeTracks);
    else
        colConfirmedTracks = [colConfirmedTracks; toStruct(confirmedTracks)]; %#ok<AGROW>
        colConfirmedTracksGlob = [colConfirmedTracksGlob; toStruct(confirmedTracksGlob)]; %#ok<AGROW>
        colTentativeTracks = [colTentativeTracks;toStruct(tentativeTracks)]; %#ok<AGROW> 
    end
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
positionListfromConfirmedTracks = getPositionAsList(colConfirmedTracks);
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

convertedTracks = convertConfTrack(colConfirmedTracks);
convertedTracksGlob = convertConfTrack(colConfirmedTracksGlob);

figure;
swarmchart(convertedTracks(:,1),...
         convertedTracks(:,2),...
         'filled','MarkerFaceAlpha',0.5,'MarkerEdgeAlpha',0.5);
axis([round(min(convertedTracks(:,1))-10),...
      round((max(convertedTracks(:,1))+10)),... %inf,inf]);
      round(min(convertedTracks(:,2))),...
      round((max(convertedTracks(:,2))))]);
axis equal


figure;
scatter(convertedTracksGlob(:,1),...
         convertedTracksGlob(:,2),...
         'filled','MarkerFaceAlpha',0.5,'MarkerEdgeAlpha',0.5);
axis([round(min(convertedTracksGlob(:,1))-10),...
      round((max(convertedTracksGlob(:,1))+10)),... %inf,inf]);
      round(min(convertedTracksGlob(:,2))),...
      round((max(convertedTracksGlob(:,2))))]);
axis equal



% tbl = table(positionListfromConfirmedTracks)
% plot3(positionListfromConfirmedTracks(numTo,1),...
%     positionListfromConfirmedTracks(numTo,2),...
%     positionListfromConfirmedTracks(numTo,3))
% figure;
% for si=1:size(lidarData,1)
%     %pcshow(lidarData{si,1}.Location)
%     pcshowpair(ego_red_10sec(si).PointClouds{1,1},...
%         ego_blue_10sec(si).PointClouds{1,1})
%     pause(0.05)
% end