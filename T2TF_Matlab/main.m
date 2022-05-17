clear
clc
%% Track-to-Track Architecture

% Create the tracker for vehicle 1
v1Tracker = trackerJPDA('TrackerIndex',1, 'DeletionThreshold', [4 4], 'AssignmentThreshold', [100 inf]); % Vehicle 1 tracker
posSelector = [1 0 0 0 0 0; 0 0 1 0 0 0];

% Define sources for each vehicle

% SourceIndex 1(int), 2(int), 5(int), 4(ext), 3(ext), 
% Vehicle 1
v1TrackerConfiguration = fuserSourceConfiguration('SourceIndex',1,'IsInternalSource',true, ...   % v1Tracker is internal to v1Fuser
    "CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v2FuserConfiguration = fuserSourceConfiguration('SourceIndex',5,'IsInternalSource',false);       % v2Fuser is external to v1Fuser
%v3FuserConfiguration = fuserSourceConfiguration('SourceIndex',6,'IsInternalSource',false);       % v3Fuser is external to v1Fuser
% Should I add v3FuserConfiguration
v1Sources = {v1TrackerConfiguration; v2FuserConfiguration};
%v1Sources = {v1TrackerConfiguration; v3FuserConfiguration};

% Vehicle 2
v2TrackerConfiguration = fuserSourceConfiguration('SourceIndex',2,'IsInternalSource',true, ...   % v2Tracker is internal to v2Fuser
    "CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v1FuserConfiguration = fuserSourceConfiguration('SourceIndex',4,'IsInternalSource',false);       % v1Fuser is external to v2Fuser
v2Sources = {v2TrackerConfiguration; v1FuserConfiguration};

% % Vehicle 3
v3TrackerConfiguration = fuserSourceConfiguration('SourceIndex',3,'IsInternalSource',true, ...   % v3Tracker is internal to v3Fuser
    "CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v1FuserConfiguration = fuserSourceConfiguration('SourceIndex',4,'IsInternalSource',false);       % v1Fuser is external to v3Fuser
v3Sources = {v3TrackerConfiguration; v1FuserConfiguration};


% Track Fuser Object
% trackFusers 3 - 4 - 5
stateParams = struct('Frame','Rectangular','Position',[0 0 0],'Velocity',[0 0 0]);
v1Fuser = trackFuser('FuserIndex',4,...
    'AssignmentThreshold', [100 inf], ...
    'MaxNumSources',2,'SourceConfigurations',v1Sources,...
    'StateFusion','Intersection','DeletionThreshold',[3 3],...
    'StateParameters',stateParams);
v2Fuser = trackFuser('FuserIndex',5,...
    'AssignmentThreshold', [100 inf], ...
    'MaxNumSources',2,'SourceConfigurations',v2Sources,'StateFusion',...
    'Intersection','DeletionThreshold',[3 3],...
    'StateParameters',stateParams);
v3Fuser = trackFuser('FuserIndex',6,...
    'AssignmentThreshold', [100 inf], ...
    'MaxNumSources',2,'SourceConfigurations',v3Sources,'StateFusion',...
    'Intersection','DeletionThreshold',[3 3],...
    'StateParameters',stateParams);

% Initialize the following variables
fusedTracks1 = objectTrack.empty(0,1);
fusedTracks2 = objectTrack.empty(0,1);
fusedTracks3 = objectTrack.empty(0,1);
wasFuser1Updated = false;
wasFuser2Updated = false;
wasFuser3Updated = false;



%% Define Scenario

% Create the drivingScenario object and the two vehicles
[scenario, vehicle1, vehicle2, vehicle3] = createDrivingScenario;

% Create all the sensors
[sensors, numSensors, attachedVehicle] = createSensors(scenario);

% Create display
[f,plotters] = createV2VDisplay(scenario, sensors, attachedVehicle);

% Define each vehicle as a combination of an actor, sensors, a tracker, and plotters
v1=struct('Actor',{vehicle1},'Sensors',{sensors(attachedVehicle==1)},'Tracker',{v1Tracker},'DetPlotter',{plotters.veh1DetPlotter},'TrkPlotter',{plotters.veh1TrkPlotter});
v2=struct('Actor',{vehicle2},'Sensors',{sensors(attachedVehicle==2)},'Tracker',{{}},'DetPlotter',{{}},'TrkPlotter',{plotters.veh2TrkPlotter}); % No detections or tracker on Vehicle 2
v3=struct('Actor',{vehicle3},'Sensors',{sensors(attachedVehicle==3)},'Tracker',{{}},'DetPlotter',{{}},'TrkPlotter',{plotters.veh3TrkPlotter}); % No detections or tracker on Vehicle 3

%% Run Simulation

running = true;

% For repeatable results, set the random number seed
s = rng;
rng(2019)
snaptimes = [0.5, 2.8, 4.4, 6.3, inf];
snaps = cell(numel(snaptimes,1));
i = 1;
f.Visible = 'on';
while running && ishghandle(f)
    time  = scenario.SimulationTime;

    % Detect and track at the vehicle level
    [tracks1,wasTracker1Updated,detections1] = detectAndTrack(v1,time,posSelector);
    [tracks2,wasTracker2Updated,detections2] = detectAndTrack(v2,time,posSelector);
    [tracks3,wasTracker3Updated,detections3] = detectAndTrack(v3,time,posSelector);

    % Keep the tracks from the previous fuser update
    oldFusedTracks1 = fusedTracks1; % Why not immediately transfers the info?
    oldFusedTracks2 = fusedTracks2;
    oldFusedTracks3 = fusedTracks3;


    % Update the fusers
    % Vehicle 1
    if wasTracker1Updated || wasFuser2Updated
        tracksToFuse1 = [tracks1;oldFusedTracks2];
        if isLocked(v1Fuser) || ~isempty(tracksToFuse1)
            [fusedTracks1,~,~,info1] = v1Fuser(tracksToFuse1,time); %% GNN Model applied here, single-hypothesis track-2-track fuser
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
        tracksToFuse2 = [tracks2;oldFusedTracks1];
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
        tracksToFuse3 = [tracks3;oldFusedTracks1];
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



    % Update the display
    updateV2VDisplay(plotters, scenario, sensors, attachedVehicle)

    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);

    % Capture an image of the frame at specified times
    if time >= snaptimes(i)
        snaps{i} = takesnap(f);
        i = i + 1;
    end
end

%% Analyze 
% Tracking at Beginning of Simulation
% showsnap(snaps,1)
% 
% % Tracking of Pedestrian at Side of Street
% showsnap(snaps,2)
% showsnap(snaps,3)
% 
% %Avoiding Rumor Propagation
% showsnap(snaps,4)

% Restart the driving scenario to return the actors to their initial positions
restart(scenario);

% Release all the sensor objects so they can be used again
for sensorIndex = 1:numSensors
    release(sensors{sensorIndex});
end

% Return the random seed to its previous value
rng(s)