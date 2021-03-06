%% main
clear
%% TRACKER DESIGN
% Create the tracker for vehicle 1
    % 'TrackerIndex',1, distungiushes tracks that come from different trackers 
    % 'AssignmentThreshold', [Max#ofTrack Max#ofDetection]
    % 'DeletionThreshold', [4 4], [P R]. If a confirmed track is not assigned
    %  to any detection P times in the last R tracker updates, then the track is deleted. P must be less than or equal to R.

v1Tracker = trackerJPDA('TrackerIndex',1, 'DeletionThreshold', [4 4], 'AssignmentThreshold', [100 inf]); % Vehicle 1 tracker
posSelector = [1 0 0 0 0 0; 0 0 1 0 0 0];

% Define sources for vehicle 1
v1TrackerConfiguration = fuserSourceConfiguration('SourceIndex',1,'IsInternalSource',true, ...   % v1Tracker is internal to v1Fuser
    "CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v2FuserConfiguration = fuserSourceConfiguration('SourceIndex',4,'IsInternalSource',false);       % v2Fuser is external to v2Fuser
v1Sources = {v1TrackerConfiguration; v2FuserConfiguration};

% Define sources for vehicle 2
% Source 1: Internal
v2TrackerConfiguration = fuserSourceConfiguration('SourceIndex',2,'IsInternalSource',true, ...   % v2Tracker is internal to v2Fuser
    "CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v1FuserConfiguration = fuserSourceConfiguration('SourceIndex',3,'IsInternalSource',false);       % v1Fuser is external to v2Fuser
v2Sources = {v2TrackerConfiguration; v1FuserConfiguration};

% Define trackFuser object for each vehicle
stateParams = struct('Frame','Rectangular','Position',[0 0 0],'Velocity',[0 0 0]); %Initial STATE
v1Fuser = trackFuser('FuserIndex',3,...
    'AssignmentThreshold', [100 inf], ...
    'MaxNumSources',2,'SourceConfigurations',v1Sources,...
    'StateFusion','Intersection','DeletionThreshold',[3 3],...
    'StateParameters',stateParams);
v2Fuser = trackFuser('FuserIndex',4,...
    'AssignmentThreshold', [100 inf], ...
    'MaxNumSources',2,'SourceConfigurations',v2Sources,'StateFusion',...
    'Intersection','DeletionThreshold',[3 3],...
    'StateParameters',stateParams);

% Initialize the following variables
fusedTracks1 = objectTrack.empty(0,1);
fusedTracks2 = objectTrack.empty(0,1);
wasFuser1Updated = false;
wasFuser2Updated = false;

% Create the drivingScenario object and the two vehicles
[scenario, vehicle1, vehicle2] = createDrivingScenario;

% Create all the sensors
[sensors, numSensors, attachedVehicle] = createSensors(scenario);

% Create display
[f,plotters] = createV2VDisplay(scenario, sensors, attachedVehicle);

% Define each vehicle as a combination of an actor, sensors, a tracker, and plotters
v1=struct('Actor',{vehicle1},'Sensors',{sensors(attachedVehicle==1)},'Tracker',{v1Tracker},'DetPlotter',{plotters.veh1DetPlotter},'TrkPlotter',{plotters.veh1TrkPlotter});
v2=struct('Actor',{vehicle2},'Sensors',{sensors(attachedVehicle==2)},'Tracker',{{}},'DetPlotter',{{}},'TrkPlotter',{plotters.veh2TrkPlotter}); % No detections or tracker on Vehicle 2

%% Simulation Running
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
    [tracks1,wasTracker1Updated] = detectAndTrack(v1,time,posSelector);
    [tracks2,wasTracker2Updated] = detectAndTrack(v2,time,posSelector);

    % Keep the tracks from the previous fuser update
    oldFusedTracks1 = fusedTracks1;
    oldFusedTracks2 = fusedTracks2;

    % Update the fusers
    if wasTracker1Updated || wasFuser2Updated
        tracksToFuse1 = [tracks1;oldFusedTracks2];
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
showsnap(snaps,1)
showsnap(snaps,2)
showsnap(snaps,3)
showsnap(snaps,4)

%% Restarting
% Restart the driving scenario to return the actors to their initial positions
restart(scenario);

% Release all the sensor objects so they can be used again
for sensorIndex = 1:numSensors
    release(sensors{sensorIndex});
end

% Return the random seed to its previous value
rng(s)
