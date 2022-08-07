% Clear
clear
clc
close all

%% Configuration
ta = trackingArchitecture;

% Fuser Configurations
v1FuserConfiguration = fuserSourceConfiguration('SourceIndex',21,'IsInternalSource',false);       % v1Fuser is external to v2Fuser
v2FuserConfiguration = fuserSourceConfiguration('SourceIndex',22,'IsInternalSource',false);       % v2Fuser is external to v1Fuser
v3FuserConfiguration = fuserSourceConfiguration('SourceIndex',23,'IsInternalSource',false);       % v3Fuser is external to v1Fuser

% Vehicle 1
v1TrackerConfiguration = fuserSourceConfiguration('SourceIndex',10,'IsInternalSource',true);   % v1Tracker is internal to v1Fuser
    %"CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v1Sources = {v1TrackerConfiguration; v2FuserConfiguration; v3FuserConfiguration};

% Vehicle 2
v2TrackerConfiguration = fuserSourceConfiguration('SourceIndex',11,'IsInternalSource',true);   % v2Tracker is internal to v2Fuser
    %"CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v2Sources = {v2TrackerConfiguration; v1FuserConfiguration; v3FuserConfiguration};

% Vehicle 3
v3TrackerConfiguration = fuserSourceConfiguration('SourceIndex',12,'IsInternalSource',true);   % v3Tracker is internal to v3Fuser
    %"CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v3Sources = {v3TrackerConfiguration; v1FuserConfiguration; v2FuserConfiguration};

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

addTracker(ta, v1Tracker, 'SensorIndices',[1,2],'ToOutput',false);
addTracker(ta, v2Tracker, 'SensorIndices',[3,4],'ToOutput',false);
addTracker(ta, v3Tracker, 'SensorIndices',[5,6],'ToOutput',false);
addTrackFuser(ta,v1Fuser);
addTrackFuser(ta,v2Fuser);
addTrackFuser(ta,v3Fuser);

figure
show(ta)

%% Initialize the following variables
fusedTracks1 = objectTrack.empty(0,1);
fusedTracks2 = objectTrack.empty(0,1);
fusedTracks3 = objectTrack.empty(0,1);
wasFuser1Updated = false;
wasFuser2Updated = false;
wasFuser3Updated = false;

%% Define Scenario, Sensors and Display
[scenario, vehicle1, vehicle2, vehicle3] = createDrivingScenario;
[sensors, numSensors, attachedVehicle] = createSensors(scenario);
[f,plotters] = createV2VDisplay(scenario, sensors, attachedVehicle);
f.Visible = 'on';

% Define each vehicle as a combination of an actor, sensors, a tracker, and plotters
v1=struct('Actor',{vehicle1},'Sensors',{sensors(attachedVehicle==1)},'Tracker',{v1Tracker},'DetPlotter',{plotters.veh1DetPlotter},'TrkPlotter',{plotters.veh1TrkPlotter});
v2=struct('Actor',{vehicle2},'Sensors',{sensors(attachedVehicle==2)},'Tracker',{v2Tracker},'DetPlotter',{plotters.veh2DetPlotter},'TrkPlotter',{plotters.veh2TrkPlotter});
v3=struct('Actor',{vehicle3},'Sensors',{sensors(attachedVehicle==3)},'Tracker',{v3Tracker},'DetPlotter',{plotters.veh3DetPlotter},'TrkPlotter',{plotters.veh3TrkPlotter}); 

%% Run Simulation
running = true;

% For repeatable results, set the random number seed
s = rng;
rng(2019)
i = 1;

restart(scenario)

while running && ishghandle(f)
    time  = scenario.SimulationTime;

    % Detect and track at the vehicle body frame
    [tracks1,wasTracker1Updated,detections1] = detectAndTrack(v1,time);
    [tracks2,wasTracker2Updated,detections2] = detectAndTrack(v2,time);
    [tracks3,wasTracker3Updated,detections3] = detectAndTrack(v3,time);
    
%     tracksInScenario1 = egoToScenario(tracks1);
%     tracksInScenario2 = egoToScenario(tracks2);
%     tracksInScenario3 = egoToScenario(tracks3);

    % Keep the tracks from the previous fuser update
    oldFusedTracks1 = fusedTracks1;
    oldFusedTracks2 = fusedTracks2;
    oldFusedTracks3 = fusedTracks3;

    % Update the fusers
    % Vehicle 1s
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
       
    % Vehicle 2
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
    
    % Vehicle 3
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
    
    % Convert the Detections and the Tracks in Scenario Frame
    

    % Update the display
    %plotDetectionsAndTracks(agent.DetPlotter, detPos, agent.TrkPlotter, trkPos);
    updateV2VDisplay(plotters, scenario, sensors, attachedVehicle)
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
    for plotIndex = 1:numel(tracks1)
        trackState3 = [tracks1(plotIndex,1).State(1),...
                      tracks1(plotIndex,1).State(3)]; 
        plotTrack(v3.TrkPlotter,trackState3);
    end
    
    for detIndex = 1:numel(detections1)
        detection1InScenario = egoToScenario(v1.Actor,detections1{detIndex, 1});
        plotDetection(plotters.veh1DetPlotter,detection1InScenario(1:2)')  
    end
    for detIndex = 1:numel(detections2)
        detection2InScenario = egoToScenario(v2.Actor,detections2{detIndex, 1});
        plotDetection(plotters.veh2DetPlotter,detection2InScenario(1:2)')  
    end
    for detIndex = 1:numel(detections3)
        detection3InScenario = egoToScenario(v3.Actor,detections3{detIndex, 1});
        plotDetection(plotters.veh3DetPlotter,detection3InScenario(1:2)')  
    end
    running = advance(scenario);

end








