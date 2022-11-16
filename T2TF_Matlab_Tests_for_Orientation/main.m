% Clear
clear
clc
close all

%% Configuration
ta = trackingArchitecture;
ta2 = trackingArchitecture;

manipulationIsOn = false;

% Fuser Configurations
v1FuserConfiguration = fuserSourceConfiguration('SourceIndex',21,'IsInternalSource',false);       % v1Fuser is external to v2Fuser
v2FuserConfiguration = fuserSourceConfiguration('SourceIndex',22,'IsInternalSource',false);       % v2Fuser is external to v1Fuser
v3FuserConfiguration = fuserSourceConfiguration('SourceIndex',23,'IsInternalSource',false);       % v3Fuser is external to v1Fuser

% Manipulated Fuser Configurations
v1ManipulatedFuserConfiguration = fuserSourceConfiguration('SourceIndex',921,'IsInternalSource',false);       % v1Fuser is external to v2Fuser
v2ManipulatedFuserConfiguration = fuserSourceConfiguration('SourceIndex',922,'IsInternalSource',false);       % v2Fuser is external to v1Fuser
v3ManipulatedFuserConfiguration = fuserSourceConfiguration('SourceIndex',923,'IsInternalSource',false);       % v3Fuser is external to v1Fuser


% Vehicle 1
v1TrackerConfiguration = fuserSourceConfiguration('SourceIndex',10,'IsInternalSource',true);   % v1Tracker is internal to v1Fuser
    %"CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v1Sources = {v1TrackerConfiguration; v2FuserConfiguration; v3FuserConfiguration};

v1ManipulatedTrackerConfiguration = fuserSourceConfiguration('SourceIndex',100,'IsInternalSource',true);   % v1Tracker is internal to v1Fuser
    %"CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v1ManipulatedSources = {v1ManipulatedTrackerConfiguration; v2ManipulatedFuserConfiguration; v3ManipulatedFuserConfiguration};


% Vehicle 2
v2TrackerConfiguration = fuserSourceConfiguration('SourceIndex',11,'IsInternalSource',true);   % v2Tracker is internal to v2Fuser
    %"CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v2Sources = {v2TrackerConfiguration; v1FuserConfiguration; v3FuserConfiguration};

v2ManipulatedTrackerConfiguration = fuserSourceConfiguration('SourceIndex',101,'IsInternalSource',true);   % v2Tracker is internal to v2Fuser
    %"CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v2ManipulatedSources = {v2ManipulatedTrackerConfiguration; v1ManipulatedFuserConfiguration; v3ManipulatedFuserConfiguration};


% Vehicle 3
v3TrackerConfiguration = fuserSourceConfiguration('SourceIndex',12,'IsInternalSource',true);   % v3Tracker is internal to v3Fuser
    %"CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v3Sources = {v3TrackerConfiguration; v1FuserConfiguration; v2FuserConfiguration};

v3ManipulatedTrackerConfiguration = fuserSourceConfiguration('SourceIndex',102,'IsInternalSource',true);   % v3Tracker is internal to v3Fuser
    %"CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v3ManipulatedSources = {v3ManipulatedTrackerConfiguration; v1ManipulatedFuserConfiguration; v2ManipulatedFuserConfiguration};

%% Track Fuser Object Generation
posSelector = [1 0 0 0 0 0; 0 0 1 0 0 0];
stateParams = struct('Frame','Rectangular','Position',[0 0 0],'Velocity',[0 0 0]);

% Tracker for V1 radar and camera sensors 
v1Tracker = trackerJPDA('TrackerIndex',10, 'DeletionThreshold', [4 4],...
     'AssignmentThreshold', [100 Inf],'TrackLogic','History'); % Vehicle 1 tracker
v2Tracker = trackerJPDA('TrackerIndex',11, 'DeletionThreshold', [4 4],...
     'AssignmentThreshold', [100 Inf],'TrackLogic','History'); % Vehicle 1 tracker
v3Tracker = trackerJPDA('TrackerIndex',12, 'DeletionThreshold', [4 4],...
     'AssignmentThreshold', [100 Inf],'TrackLogic','History'); % Vehicle 1 tracker


v1Fuser = trackFuser('FuserIndex',21,...
    'Assignment','Auction',...
    'AssignmentThreshold', [100 inf], ...
    'MaxNumSources',3,'SourceConfigurations',v1Sources,...
    'StateFusion','Intersection','DeletionThreshold',[2 2],...
    'StateParameters',stateParams);
v2Fuser = trackFuser('FuserIndex',22,...
    'Assignment','Auction',...
    'AssignmentThreshold', [100 inf], ...
    'MaxNumSources',3,'SourceConfigurations',v2Sources,'StateFusion',...
    'Intersection','DeletionThreshold',[2 2],...
    'StateParameters',stateParams);
v3Fuser = trackFuser('FuserIndex',23,...
    'Assignment','Auction',...
    'AssignmentThreshold', [100 inf], ...
    'MaxNumSources',3,'SourceConfigurations',v3Sources,'StateFusion',...
    'Intersection','DeletionThreshold',[2 2],...
    'StateParameters',stateParams);

% ManipulatedTracker for V1 radar and camera sensors 
v1ManipulatedTracker = trackerJPDA('TrackerIndex',100, 'DeletionThreshold', [4 4],...
     'AssignmentThreshold', [100 Inf],'TrackLogic','History'); % Vehicle 1 tracker
v2ManipulatedTracker = trackerJPDA('TrackerIndex',101, 'DeletionThreshold', [4 4],...
     'AssignmentThreshold', [100 Inf],'TrackLogic','History'); % Vehicle 1 tracker
v3ManipulatedTracker = trackerJPDA('TrackerIndex',102, 'DeletionThreshold', [4 4],...
     'AssignmentThreshold', [100 Inf],'TrackLogic','History'); % Vehicle 1 tracker

v1ManipulatedFuser = trackFuser('FuserIndex',921,...
    'Assignment','Auction',...
    'AssignmentThreshold', [100 inf], ...
    'MaxNumSources',3,'SourceConfigurations',v1ManipulatedSources,...
    'StateFusion','Intersection','DeletionThreshold',[2 2],...
    'StateParameters',stateParams);
v2ManipulatedFuser = trackFuser('FuserIndex',922,...
    'Assignment','Auction',...
    'AssignmentThreshold', [100 inf], ...
    'MaxNumSources',3,'SourceConfigurations',v2ManipulatedSources,'StateFusion',...
    'Intersection','DeletionThreshold',[2 2],...
    'StateParameters',stateParams);
v3ManipulatedFuser = trackFuser('FuserIndex',923,...
    'Assignment','Auction',...
    'AssignmentThreshold', [100 inf], ...
    'MaxNumSources',3,'SourceConfigurations',v3ManipulatedSources,'StateFusion',...
    'Intersection','DeletionThreshold',[2 2],...
    'StateParameters',stateParams);

addTracker(ta, v1Tracker, 'SensorIndices',[1,2],'ToOutput',false);
addTracker(ta, v2Tracker, 'SensorIndices',[3,4],'ToOutput',false);
addTracker(ta, v3Tracker, 'SensorIndices',[5,6],'ToOutput',false);
addTrackFuser(ta,v1Fuser);
addTrackFuser(ta,v2Fuser);
addTrackFuser(ta,v3Fuser);


% addTracker(ta2, v1ManipulatedTracker, 'SensorIndices',[501,502],'ToOutput',false);
% addTracker(ta2, v2ManipulatedTracker, 'SensorIndices',[503,504],'ToOutput',false);
% addTracker(ta2, v3ManipulatedTracker, 'SensorIndices',[505,506],'ToOutput',false);
% addTrackFuser(ta2,v1ManipulatedFuser);
% addTrackFuser(ta2,v2ManipulatedFuser);
% addTrackFuser(ta2,v3ManipulatedFuser);

figure
show(ta)
% figure
% show(ta2)

%% Initialize the following variables
fusedTracks1 = objectTrack.empty(0,1);
fusedTracks2 = objectTrack.empty(0,1);
fusedTracks3 = objectTrack.empty(0,1);

manipulatedFusedTracks1 = objectTrack.empty(0,1);
manipulatedFusedTracks2 = objectTrack.empty(0,1);
manipulatedFusedTracks3 = objectTrack.empty(0,1);

wasFuser1Updated = false;
wasFuser2Updated = false;
wasFuser3Updated = false;

wasManipulatedFuser1Updated = false;
wasManipulatedFuser2Updated = false;
wasManipulatedFuser3Updated = false;

%% Define Scenario, Sensors and Display
[scenario, vehicle1, vehicle2, vehicle3] = createDrivingScenario;
[sensors, numSensors, attachedVehicle] = createSensors(scenario);
[f,plotters] = createV2VDisplay(scenario, sensors, attachedVehicle);
f.Visible = 'on';

% Define each vehicle as a combination of an actor, sensors, a tracker, and plotters
v1=struct('Actor',{vehicle1},'Sensors',{sensors(attachedVehicle==1)},'Manipulated',{[2]},...
    'Tracker',{v1Tracker},'ManipulatedTracker',{v1ManipulatedTracker},...
    'DetPlotter',{plotters.veh1DetPlotter},'ManDetPlotter',{plotters.veh1ManDetPlotter},...
    'TrkPlotter',{plotters.veh1TrkPlotter},'ManTrkPlotter',{plotters.veh1ManTrkPlotter});

v2=struct('Actor',{vehicle2},'Sensors',{sensors(attachedVehicle==2)},'Manipulated',{[]},...
    'Tracker',{v2Tracker},'ManipulatedTracker',{v2ManipulatedTracker},...
    'DetPlotter',{plotters.veh2DetPlotter},'ManDetPlotter',{plotters.veh2ManDetPlotter},...
    'TrkPlotter',{plotters.veh2TrkPlotter},'ManTrkPlotter',{plotters.veh2ManTrkPlotter});

v3=struct('Actor',{vehicle3},'Sensors',{sensors(attachedVehicle==3)},'Manipulated',{[]},...
    'Tracker',{v3Tracker},'ManipulatedTracker',{v3ManipulatedTracker},...
    'DetPlotter',{plotters.veh3DetPlotter},'ManDetPlotter',{plotters.veh3ManDetPlotter},...
    'TrkPlotter',{plotters.veh3TrkPlotter},'ManTrkPlotter',{plotters.veh3ManTrkPlotter}); 

%% Run Simulation
running = true;

% For repeatable results, set the random number seed
s = rng;
rng(2019)
i = 1;

restart(scenario)

while running && ishghandle(f)
    manipulatedSensor= ['v1', 'sensor1'];
    time  = scenario.SimulationTime;
    timeStr = 'T' + string(time);
    timeStr = strrep(timeStr,'.','_');
    allData.(timeStr).detections1InEgo = struct();
    allData.(timeStr).detections2InEgo = struct();
    allData.(timeStr).detections3InEgo = struct();
    allData.(timeStr).detection1InScenario = struct();
    allData.(timeStr).detection2InScenario = struct();
    allData.(timeStr).detection3InScenario = struct();
    allData.(timeStr).tracks1 = struct();
    allData.(timeStr).tracks2 = struct();
    allData.(timeStr).tracks3 = struct();
    allData.(timeStr).fusedTrack1 = struct();
    allData.(timeStr).fusedTrack2 = struct();
    allData.(timeStr).fusedTrack3 = struct();

    % Detect and track at the vehicle body frame
    [tracks1,manipulatedTracks1, wasTracker1Updated,detections1, manipulatedDetection1] = detectAndTrack(v1,time);
    [tracks2,manipulatedTracks2, wasTracker2Updated,detections2, manipulatedDetection2] = detectAndTrack(v2,time);
    [tracks3,manipulatedTracks3, wasTracker3Updated,detections3, manipulatedDetection3] = detectAndTrack(v3,time);
    allData.(timeStr).detections1InEgo = getState(detections1,'Measurement');
    allData.(timeStr).detections2InEgo = getState(detections2,'Measurement');
    allData.(timeStr).detections3InEgo = getState(detections3,'Measurement');
    allData.(timeStr).manipulatedDetections1InEgo = getState(manipulatedDetection1,'Measurement');
    allData.(timeStr).manipulatedDetections2InEgo = getState(manipulatedDetection2,'Measurement');
    allData.(timeStr).manipulatedDetections3InEgo = getState(manipulatedDetection3,'Measurement');
    allData.(timeStr).tracks1 = getState(tracks1,'State');
    allData.(timeStr).tracks2 = getState(tracks2,'State');
    allData.(timeStr).tracks3 = getState(tracks3,'State');
    allData.(timeStr).manipulatedTracks1 = getState(manipulatedTracks1,'State');
    allData.(timeStr).manipulatedTracks2 = getState(manipulatedTracks2,'State');
    allData.(timeStr).manipulatedTracks3 = getState(manipulatedTracks3,'State');

%     tracksInScenario1 = egoToScenario(tracks1);
%     tracksInScenario2 = egoToScenario(tracks2);
%     tracksInScenario3 = egoToScenario(tracks3);

    % Keep the tracks from the previous fuser update
    oldFusedTracks1 = fusedTracks1;
    oldFusedTracks2 = fusedTracks2;
    oldFusedTracks3 = fusedTracks3;

    oldManipulatedFusedTracks1 = manipulatedFusedTracks1; 
    oldManipulatedFusedTracks2 = manipulatedFusedTracks2;
    oldManipulatedFusedTracks3 = manipulatedFusedTracks3;

    % Update the fusers
    % Vehicle 1s
    % v1Tracker
    if wasTracker1Updated || wasFuser2Updated
        tracksToFuse1 = [tracks1;oldFusedTracks2;oldFusedTracks3];
        if isLocked(v1Fuser) || ~isempty(tracksToFuse1)
            [fusedTracks1,~,~,info1] = v1Fuser(tracksToFuse1,time);
            
            wasFuser1Updated = true;
            pos = getTrackPositions(fusedTracks1,posSelector);
            ids = string([fusedTracks1.TrackID]');
            plotTrack(plotters.veh1FusePlotter,pos,ids);            
        else
            wasFuser1Updated = false;
            fusedTracks1 = objectTrack.empty(0,1);
            
        end
    else
        wasFuser1Updated = false;
        fusedTracks1 = objectTrack.empty(0,1);

    end
    % v1ManipulatedTracker
    if manipulationIsOn == true
        if wasTracker1Updated || wasFuser2Updated
            manipulatedTracksToFuse1 = [manipulatedTracks1; oldManipulatedFusedTracks2; oldManipulatedFusedTracks3];
            if isLocked(v1ManipulatedFuser) || ~isempty(manipulatedTracksToFuse1)
                [manipulatedFusedTracks1,~,~,manipulatedInfo1] = v1ManipulatedFuser(manipulatedTracksToFuse1,time);
    
                wasManipulatedFuser1Updated = true;            
                manipulatedPos = getTrackPositions(manipulatedFusedTracks1,posSelector);
                manipulatedIds = string([manipulatedFusedTracks1.TrackID]');
                plotTrack(plotters.veh1ManFusePlotter,manipulatedPos,manipulatedIds);
            else
                wasManipulatedFuser1Updated = false;
                manipulatedFusedTracks1 = objectTrack.empty(0,1);
            end
        else
            wasManipulatedFuser1Updated = false;
            manipulatedFusedTracks1 = objectTrack.empty(0,1);
        end
    end
       
    % Vehicle 2
    % v2Tracker    
    if wasTracker2Updated || wasFuser1Updated
        tracksToFuse2 = [tracks2;oldFusedTracks1;oldFusedTracks3];
        if isLocked(v2Fuser) || ~isempty(tracksToFuse2)
            [fusedTracks2,~,~,info2] = v2Fuser(tracksToFuse2,time);
            wasFuser2Updated = true;
            pos = getTrackPositions(fusedTracks2,posSelector);
            ids = string([fusedTracks2.TrackID]');
            plotTrack(plotters.veh2FusePlotter,pos,ids);
        else
            wasFuser2Updated = false;
            fusedTracks2 = objectTrack.empty(0,1);
        end
    else
        wasFuser2Updated = false;
        fusedTracks2 = objectTrack.empty(0,1);
    end
    % v2ManipulatedTracker    
    if manipulationIsOn == true
        if wasTracker2Updated || wasFuser1Updated
            manipulatedTracksToFuse2 = [manipulatedTracks2;oldManipulatedFusedTracks1;oldManipulatedFusedTracks3];
            if isLocked(v2ManipulatedFuser) || ~isempty(manipulatedTracksToFuse2)
                [manipulatedFusedTracks2,~,~,manipulatedInfo2] = v2ManipulatedFuser(manipulatedTracksToFuse2,time);
                
                wasManipulatedFuser2Updated = true;
                manipulatedPos = getTrackPositions(manipulatedFusedTracks2,posSelector);
                manipulatedIds = string([manipulatedFusedTracks2.TrackID]');
                plotTrack(plotters.veh2ManFusePlotter,manipulatedPos,manipulatedIds);
            else
                wasManipulatedFuser2Updated = false;
                manipulatedFusedTracks2 = objectTrack.empty(0,1);
            end
        else
            wasManipulatedFuser2Updated = false;
            manipulatedFusedTracks2 = objectTrack.empty(0,1);
        end
    end
    
    % Vehicle 3
    % v3Tracker    
    if wasTracker3Updated || wasFuser1Updated
        tracksToFuse3 = [tracks3;oldFusedTracks1;oldFusedTracks2];
        if isLocked(v3Fuser) || ~isempty(tracksToFuse3)
            [fusedTracks3,~,~,info3] = v3Fuser(tracksToFuse3,time); %% error
            wasFuser3Updated = true;
            pos = getTrackPositions(fusedTracks3,posSelector);
            ids = string([fusedTracks3.TrackID]');
            plotTrack(plotters.veh3FusePlotter,pos,ids);
        else
            wasFuser3Updated = false;
            fusedTracks3 = objectTrack.empty(0,1);
        end
    else
        wasFuser3Updated = false;
        fusedTracks3 = objectTrack.empty(0,1);
    end
    
    % v3ManipulatedTracker
    if manipulationIsOn == true
        if wasTracker3Updated || wasFuser1Updated
            manipulatedTracksToFuse3 = [manipulatedTracks3;oldManipulatedFusedTracks1;oldManipulatedFusedTracks2];
            if isLocked(v3ManipulatedFuser) || ~isempty(manipulatedTracksToFuse3)
                [manipulatedFusedTracks3,~,~,manipulatedInfo3] = v3ManipulatedFuser(manipulatedTracksToFuse3,time); %% error
                
                wasManipulatedFuser3Updated = true;
                manipulatedPos = getTrackPositions(manipulatedFusedTracks3,posSelector);
                manipulatedIds = string([manipulatedFusedTracks3.TrackID]');
                plotTrack(plotters.veh3ManFusePlotter,manipulatedPos,manipulatedIds);
            else
                wasManipulatedFuser3Updated = false;
                manipulatedFusedTracks3 = objectTrack.empty(0,1);
            end
        else
            wasManipulatedFuser3Updated = false;
            manipulatedFusedTracks3 = objectTrack.empty(0,1);
        end
    end



    allData.(timeStr).fusedTrack1 = getState(fusedTracks1,'State');
    allData.(timeStr).fusedTrack2 = getState(fusedTracks2,'State');
    allData.(timeStr).fusedTrack3 = getState(fusedTracks3,'State');

    if manipulationIsOn == true
        allData.(timeStr).manipulatedFusedTrack1 = getState(manipulatedFusedTracks1,'State');
        allData.(timeStr).manipulatedFusedTrack2 = getState(manipulatedFusedTracks2,'State');
        allData.(timeStr).manipulatedFusedTrack3 = getState(manipulatedFusedTracks3,'State');
    end    
        % Update the display
    %plotDetectionsAndTracks(agent.DetPlotter, detPos, agent.TrkPlotter, trkPos);
    updateV2VDisplay(plotters, scenario, sensors, attachedVehicle)
    
    % Plotting Tracks
    for plotIndex = 1:numel(tracks1)
        trackState1 = [tracks1(plotIndex,1).State(1),...
                      tracks1(plotIndex,1).State(3)]; 
        plotTrack(v1.TrkPlotter,trackState1);
    end
    for plotIndex = 1:numel(tracks2)
        trackState2 = [tracks2(plotIndex,1).State(1),...
                      tracks2(plotIndex,1).State(3)]; 
        plotTrack(v2.TrkPlotter,trackState2);
    end
    for plotIndex = 1:numel(tracks3)
        trackState3 = [tracks3(plotIndex,1).State(1),...
                      tracks3(plotIndex,1).State(3)]; 
        plotTrack(v3.TrkPlotter,trackState3);
    end

    % Plotting Manipulated Tracks
    if manipulationIsOn == true
        for plotIndex = 1:numel(manipulatedTracks1)
            manipulatedTrackState1 = [manipulatedTracks1(plotIndex,1).State(1),...
                          manipulatedTracks1(plotIndex,1).State(3)]; 
            plotTrack(v1.ManTrkPlotter,manipulatedTrackState1);
        end
        for plotIndex = 1:numel(manipulatedTracks2)
            manipulatedTrackState2 = [manipulatedTracks2(plotIndex,1).State(1),...
                          manipulatedTracks2(plotIndex,1).State(3)]; 
            plotTrack(v2.ManTrkPlotter,manipulatedTrackState2);
        end
        for plotIndex = 1:numel(manipulatedTracks3)
            manipulatedTrackState3 = [manipulatedTracks3(plotIndex,1).State(1),...
                          manipulatedTracks3(plotIndex,1).State(3)]; 
            plotTrack(v3.ManTrkPlotter,manipulatedTrackState3);
        end
    end

    % Plotting Detections
    for detIndex = 1:numel(detections1)
        detection1InScenario(detIndex,:) = transpose(egoToScenario(v1.Actor,detections1{detIndex, 1}));  %#ok<SAGROW> 
    end
    plotDetection(plotters.veh1DetPlotter,detection1InScenario(:,1:2))
    
    for detIndex = 1:numel(detections2)
        detection2InScenario(detIndex,:) = transpose(egoToScenario(v2.Actor,detections2{detIndex, 1})); %#ok<SAGROW>   
    end
    plotDetection(plotters.veh2DetPlotter,detection2InScenario(:,1:2))
    
    for detIndex = 1:numel(detections3)
        detection3InScenario(detIndex,:) = transpose(egoToScenario(v3.Actor,detections3{detIndex, 1})); %#ok<SAGROW>   
    end
    plotDetection(plotters.veh3DetPlotter,detection3InScenario(:,1:2))

    % Plotting Manipulated Detections
    if manipulationIsOn == true
        for detIndex = 1:numel(manipulatedDetection1)
            manipulatedDetection1InScenario(detIndex,:) = transpose(egoToScenario(v1.Actor,manipulatedDetection1{detIndex, 1}));  %#ok<SAGROW> 
        end
        plotDetection(plotters.veh1ManDetPlotter,manipulatedDetection1InScenario(:,1:2))
        
        for detIndex = 1:numel(manipulatedDetection2)
            manipulatedDetection2InScenario(detIndex,:) = transpose(egoToScenario(v2.Actor,manipulatedDetection2{detIndex, 1})); %#ok<SAGROW>   
        end
        plotDetection(plotters.veh2ManDetPlotter,manipulatedDetection2InScenario(:,1:2))
        
        for detIndex = 1:numel(manipulatedDetection3)
            manipulatedDetection3InScenario(detIndex,:) = transpose(egoToScenario(v3.Actor,manipulatedDetection3{detIndex, 1})); %#ok<SAGROW>   
        end
        plotDetection(plotters.veh3ManDetPlotter,manipulatedDetection3InScenario(:,1:2))
    end

    allData.(timeStr).detection1InScenario = detection1InScenario;
    allData.(timeStr).detection2InScenario = detection2InScenario;
    allData.(timeStr).detection3InScenario = detection3InScenario;

    if manipulationIsOn == true
        allData.(timeStr).manipulatedDetection1InScenario = manipulatedDetection1InScenario;
        allData.(timeStr).manipulatedDetection2InScenario = manipulatedDetection2InScenario;
        allData.(timeStr).manipulatedDetection3InScenario = manipulatedDetection3InScenario;
    end

    running = advance(scenario);

end








