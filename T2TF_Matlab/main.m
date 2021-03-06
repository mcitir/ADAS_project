clear
clc
close all
%% Track-to-Track Architecture
ta = trackingArchitecture;

% Create the tracker for vehicle 1
v1Tracker = trackerJPDA('TrackerIndex',10, 'DeletionThreshold', [4 4],...
     'AssignmentThreshold', [100 Inf],'TrackLogic','History'); % Vehicle 1 tracker
posSelector = [1 0 0 0 0 0; 0 0 1 0 0 0];
addTracker(ta, v1Tracker, 'SensorIndices',[1,3],'ToOutput',false);

% Vehicle 1
v1TrackerConfiguration = fuserSourceConfiguration('SourceIndex',10,'IsInternalSource',true, ...   % v1Tracker is internal to v1Fuser
    "CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v2FuserConfiguration = fuserSourceConfiguration('SourceIndex',22,'IsInternalSource',false);       % v2Fuser is external to v1Fuser
v3FuserConfiguration = fuserSourceConfiguration('SourceIndex',23,'IsInternalSource',false);       % v3Fuser is external to v1Fuser
v1Sources = {v1TrackerConfiguration; v2FuserConfiguration; v3FuserConfiguration};
%v1Sources = {v1TrackerConfiguration; v2FuserConfiguration};
%v1Sources = {v1TrackerConfiguration; v3FuserConfiguration};


% Vehicle 2
v2TrackerConfiguration = fuserSourceConfiguration('SourceIndex',2,'IsInternalSource',true, ...   % v2Tracker is internal to v2Fuser
    "CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v1FuserConfiguration = fuserSourceConfiguration('SourceIndex',21,'IsInternalSource',false);       % v1Fuser is external to v2Fuser
v3FuserConfiguration = fuserSourceConfiguration('SourceIndex',23,'IsInternalSource',false);
v2Sources = {v2TrackerConfiguration; v1FuserConfiguration; v3FuserConfiguration};
%v2Sources = {v2TrackerConfiguration; v1FuserConfiguration};
%v2Sources = {v2TrackerConfiguration; v3FuserConfiguration};

% Vehicle 3
v3TrackerConfiguration = fuserSourceConfiguration('SourceIndex',4,'IsInternalSource',true, ...   % v3Tracker is internal to v3Fuser
    "CentralToLocalTransformFcn", @scenarioToEgo, 'LocalToCentralTransformFcn', @egoToScenario); % Coordinate transformation
v1FuserConfiguration = fuserSourceConfiguration('SourceIndex',21,'IsInternalSource',false);
v2FuserConfiguration = fuserSourceConfiguration('SourceIndex',22,'IsInternalSource',false);       % v1Fuser is external to v3Fuser
v3Sources = {v3TrackerConfiguration; v1FuserConfiguration; v2FuserConfiguration};
%v3Sources = {v3TrackerConfiguration; v1FuserConfiguration};
%v3Sources = {v3TrackerConfiguration; v2FuserConfiguration};

% Track Fuser Object
stateParams = struct('Frame','Rectangular','Position',[0 0 0],'Velocity',[0 0 0]);
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
addTrackFuser(ta,v1Fuser);
addTrackFuser(ta,v2Fuser);
addTrackFuser(ta,v3Fuser);

figure
show(ta)
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
v2=struct('Actor',{vehicle2},'Sensors',{sensors(attachedVehicle==2)},'Tracker',{{}},'DetPlotter',{plotters.veh2DetPlotter},'TrkPlotter',{plotters.veh2TrkPlotter}); % No detections or tracker on Vehicle 2
v3=struct('Actor',{vehicle3},'Sensors',{sensors(attachedVehicle==3)},'Tracker',{{}},'DetPlotter',{plotters.veh3DetPlotter},'TrkPlotter',{plotters.veh3TrkPlotter}); % No detections or tracker on Vehicle 3

%% Run Simulation

running = true;

% For repeatable results, set the random number seed
s = rng;
rng(2019)
snaptimes = [0.5, 2.8, 4.4, 6.3, inf];
snaps = cell(numel(snaptimes,1));
i = 1;
f.Visible = 'on';

truthGospa = record(scenario); % For Metrics Analysis
restart(scenario)

% Metric Analysis
tgm = trackGOSPAMetric("Distance","posabserr");

% number of timesteps: size(truthGospa,2)
% Vehicle 1
gospaLocal.vec1.lgospa       = zeros(1,size(truthGospa,2));
gospaLocal.vec1.gospa        = zeros(1,size(truthGospa,2));
gospaLocal.vec1.switching    = zeros(1,size(truthGospa,2));
gospaLocal.vec1.localization = zeros(1,size(truthGospa,2));
gospaLocal.vec1.missTarget   = zeros(1,size(truthGospa,2));
gospaLocal.vec1.falseTracks  = zeros(1,size(truthGospa,2));

gospaFused.vec1.lgospa       = zeros(1,size(truthGospa,2));
gospaFused.vec1.gospa        = zeros(1,size(truthGospa,2));
gospaFused.vec1.switching    = zeros(1,size(truthGospa,2));
gospaFused.vec1.localization = zeros(1,size(truthGospa,2));
gospaFused.vec1.missTarget   = zeros(1,size(truthGospa,2));
gospaFused.vec1.falseTracks  = zeros(1,size(truthGospa,2));

% Vehicle 2
gospaLocal.vec2.lgospa       = zeros(1,size(truthGospa,2));
gospaLocal.vec2.gospa        = zeros(1,size(truthGospa,2));
gospaLocal.vec2.switching    = zeros(1,size(truthGospa,2));
gospaLocal.vec2.localization = zeros(1,size(truthGospa,2));
gospaLocal.vec2.missTarget   = zeros(1,size(truthGospa,2));
gospaLocal.vec2.falseTracks  = zeros(1,size(truthGospa,2));

gospaFused.vec2.lgospa       = zeros(1,size(truthGospa,2));
gospaFused.vec2.gospa        = zeros(1,size(truthGospa,2));
gospaFused.vec2.switching    = zeros(1,size(truthGospa,2));
gospaFused.vec2.localization = zeros(1,size(truthGospa,2));
gospaFused.vec2.missTarget   = zeros(1,size(truthGospa,2));
gospaFused.vec2.falseTracks  = zeros(1,size(truthGospa,2));

% Vehicle 3
gospaLocal.vec3.lgospa       = zeros(1,size(truthGospa,2));
gospaLocal.vec3.gospa        = zeros(1,size(truthGospa,2));
gospaLocal.vec3.switching    = zeros(1,size(truthGospa,2));
gospaLocal.vec3.localization = zeros(1,size(truthGospa,2));
gospaLocal.vec3.missTarget   = zeros(1,size(truthGospa,2));
gospaLocal.vec3.falseTracks  = zeros(1,size(truthGospa,2));

gospaFused.vec3.lgospa       = zeros(1,size(truthGospa,2));
gospaFused.vec3.gospa        = zeros(1,size(truthGospa,2));
gospaFused.vec3.switching    = zeros(1,size(truthGospa,2));
gospaFused.vec3.localization = zeros(1,size(truthGospa,2));
gospaFused.vec3.missTarget   = zeros(1,size(truthGospa,2));
gospaFused.vec3.falseTracks  = zeros(1,size(truthGospa,2));

% gospaLocal1 = zeros(1,size(truthGospa,2)); % number of timesteps is 64
% gospaFused1 = zeros(1,size(truthGospa,2)); % number of timesteps is 64
% gospaLocal2 = zeros(1,size(truthGospa,2)); % number of timesteps is 64
% gospaFused2 = zeros(1,size(truthGospa,2)); % number of timesteps is 64
% gospaLocal3 = zeros(1,size(truthGospa,2)); % number of timesteps is 64
% gospaFused3 = zeros(1,size(truthGospa,2)); % number of timesteps is 64
iGospa = 0;

while running && ishghandle(f)
    time  = scenario.SimulationTime;
    %pause(0.2)

    % Detect and track at the vehicle level
    [tracks1,wasTracker1Updated,detections1] = detectAndTrack(v1,time,posSelector);
    [tracks2,wasTracker2Updated,detections2] = detectAndTrack(v2,time,posSelector);
    [tracks3,wasTracker3Updated,detections3] = detectAndTrack(v3,time,posSelector);

    
    %% Metrics Analysis
    % Evaluate GOSPA
    iGospa = iGospa + 1;
    
    % Gospa Metrics Vehicle1
    [gospaLocal.vec1.lgospa(iGospa),...
     gospaLocal.vec1.gospa(iGospa),...
     gospaLocal.vec1.switching(iGospa),...
     gospaLocal.vec1.localization(iGospa),...
     gospaLocal.vec1.missTarget(iGospa),...
     gospaLocal.vec1.falseTracks(iGospa)]=...
     tgm(tracks1, truthGospa(iGospa).ActorPoses);

    [gospaFused.vec1.lgospa(iGospa),...
     gospaFused.vec1.gospa(iGospa),...
     gospaFused.vec1.switching(iGospa),...
     gospaFused.vec1.localization(iGospa),...
     gospaFused.vec1.missTarget(iGospa),...
     gospaFused.vec1.falseTracks(iGospa)]=...
     tgm(fusedTracks1, truthGospa(iGospa).ActorPoses);

    % Gospa Metrics Vehicle2
    [gospaLocal.vec2.lgospa(iGospa),...
     gospaLocal.vec2.gospa(iGospa),...
     gospaLocal.vec2.switching(iGospa),...
     gospaLocal.vec2.localization(iGospa),...
     gospaLocal.vec2.missTarget(iGospa),...
     gospaLocal.vec2.falseTracks(iGospa)]=...
     tgm(tracks2, truthGospa(iGospa).ActorPoses);

    [gospaFused.vec2.lgospa(iGospa),...
     gospaFused.vec2.gospa(iGospa),...
     gospaFused.vec2.switching(iGospa),...
     gospaFused.vec2.localization(iGospa),...
     gospaFused.vec2.missTarget(iGospa),...
     gospaFused.vec2.falseTracks(iGospa)]=...
     tgm(fusedTracks2, truthGospa(iGospa).ActorPoses);

    % Gospa Metrics Vehicle3
    [gospaLocal.vec3.lgospa(iGospa),...
     gospaLocal.vec3.gospa(iGospa),...
     gospaLocal.vec3.switching(iGospa),...
     gospaLocal.vec3.localization(iGospa),...
     gospaLocal.vec3.missTarget(iGospa),...
     gospaLocal.vec3.falseTracks(iGospa)]=...
     tgm(tracks3, truthGospa(iGospa).ActorPoses);

    [gospaFused.vec3.lgospa(iGospa),...
     gospaFused.vec3.gospa(iGospa),...
     gospaFused.vec3.switching(iGospa),...
     gospaFused.vec3.localization(iGospa),...
     gospaFused.vec3.missTarget(iGospa),...
     gospaFused.vec3.falseTracks(iGospa)]=...
     tgm(fusedTracks3, truthGospa(iGospa).ActorPoses);


    % Keep the tracks from the previous fuser update
    oldFusedTracks1 = fusedTracks1; % Why not immediately transfers the info?
    oldFusedTracks2 = fusedTracks2;
    oldFusedTracks3 = fusedTracks3;


    % Update the fusers
    % Vehicle 1
    if wasTracker1Updated || wasFuser2Updated
        tracksToFuse1 = [tracks1;oldFusedTracks2;oldFusedTracks3];
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

gospaFigure=figure;
figure(gospaFigure);
tiled=tiledlayout(3,5);
title(tiled,'Generalized optimal subpattern assignment (GOSPA) metric')

% Vehicle1
nexttile
plot(gospaLocal.vec1.lgospa)
hold on
plot(gospaFused.vec1.lgospa)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec1.lgospa,gospaFused.vec1.lgospa])-2,...
      max([gospaLocal.vec1.lgospa,gospaFused.vec1.lgospa])+2])
subtitle('GOSPA with Switchin Pen.')

nexttile
plot(gospaLocal.vec1.gospa)
hold on
plot(gospaFused.vec1.gospa)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec1.gospa,gospaFused.vec1.gospa])-2,...
      max([gospaLocal.vec1.gospa,gospaFused.vec1.gospa])+2])
subtitle('GOSPA')

nexttile
plot(gospaLocal.vec1.localization)
hold on
plot(gospaFused.vec1.localization)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec1.localization,gospaFused.vec1.localization])-2,...
      max([gospaLocal.vec1.localization,gospaFused.vec1.localization])+2])
subtitle('Localization')
title('Vehicle 1')

nexttile
plot(gospaLocal.vec1.falseTracks)
hold on
plot(gospaFused.vec1.falseTracks)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec1.falseTracks,gospaFused.vec1.falseTracks])-2,...
      max([gospaLocal.vec1.falseTracks,gospaFused.vec1.falseTracks])+2])
subtitle('False Tracks')

nexttile
plot(gospaLocal.vec1.missTarget)
hold on
plot(gospaFused.vec1.missTarget)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec1.missTarget,gospaFused.vec1.missTarget])-2,...
      max([gospaLocal.vec1.missTarget,gospaFused.vec1.missTarget])+2])
subtitle('Miss Target')

% Vehicle2
nexttile
plot(gospaLocal.vec2.lgospa)
hold on
plot(gospaFused.vec2.lgospa)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec2.lgospa,gospaFused.vec2.lgospa])-2,...
      max([gospaLocal.vec2.lgospa,gospaFused.vec2.lgospa])+2])
subtitle('GOSPA with Switchin Pen.')

nexttile
plot(gospaLocal.vec2.gospa)
hold on
plot(gospaFused.vec2.gospa)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec2.gospa,gospaFused.vec2.gospa])-2,...
      max([gospaLocal.vec2.gospa,gospaFused.vec2.gospa])+2])
subtitle('GOSPA')

nexttile
plot(gospaLocal.vec2.localization)
hold on
plot(gospaFused.vec2.localization)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec2.localization,gospaFused.vec2.localization])-2,...
      max([gospaLocal.vec2.localization,gospaFused.vec2.localization])+2])
subtitle('Localization')
title('Vehicle 2')

nexttile
plot(gospaLocal.vec2.falseTracks)
hold on
plot(gospaFused.vec2.falseTracks)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec2.falseTracks,gospaFused.vec2.falseTracks])-2,...
      max([gospaLocal.vec2.falseTracks,gospaFused.vec2.falseTracks])+2])
subtitle('False Tracks')

nexttile
plot(gospaLocal.vec2.missTarget)
hold on
plot(gospaFused.vec2.missTarget)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec2.missTarget,gospaFused.vec2.missTarget])-2,...
      max([gospaLocal.vec2.missTarget,gospaFused.vec2.missTarget])+2])
subtitle('Miss Target')

% Vehicle3
nexttile
plot(gospaLocal.vec3.lgospa)
hold on
plot(gospaFused.vec3.lgospa)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec3.lgospa,gospaFused.vec3.lgospa])-2,...
      max([gospaLocal.vec3.lgospa,gospaFused.vec3.lgospa])+2])
subtitle('GOSPA with Switchin Pen.')

nexttile
plot(gospaLocal.vec3.gospa)
hold on
plot(gospaFused.vec3.gospa)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec3.gospa,gospaFused.vec3.gospa])-2,...
      max([gospaLocal.vec3.gospa,gospaFused.vec3.gospa])+2])
subtitle('GOSPA')

nexttile
plot(gospaLocal.vec3.localization)
hold on
plot(gospaFused.vec3.localization)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec3.localization,gospaFused.vec3.localization])-2,...
      max([gospaLocal.vec3.localization,gospaFused.vec3.localization])+2])
subtitle('Localization')
title('Vehicle 3')

nexttile
plot(gospaLocal.vec3.falseTracks)
hold on
plot(gospaFused.vec3.falseTracks)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec3.falseTracks,gospaFused.vec3.falseTracks])-2,...
      max([gospaLocal.vec3.falseTracks,gospaFused.vec3.falseTracks])+2])
subtitle('False Tracks')

nexttile
plot(gospaLocal.vec3.missTarget)
hold on
plot(gospaFused.vec3.missTarget)
legend({'Local Tracks','Fused Tracks'},'Location','southwest')
ylim([min([gospaLocal.vec3.missTarget,gospaFused.vec3.missTarget])-2,...
      max([gospaLocal.vec3.missTarget,gospaFused.vec3.missTarget])+2])
subtitle('Miss Target')


% subplot(3,1,2)
% plot(gospaLocal2)
% hold on
% plot(gospaFused2)
% legend({'Local Tracks','Fused Tracks'},'Location','southwest')
% %ylim([min([gospaLocal2, gospaFused2])-2,max([gospaLocal2, gospaFused2])+2])
% ylim([min([gospaLocal1, gospaFused1, ...
%            gospaLocal2, gospaFused2, ...
%            gospaLocal3, gospaFused3])-2,...
%       max([gospaLocal1, gospaFused1, ...
%            gospaLocal2, gospaFused2, ...
%            gospaLocal3, gospaFused3])+2])
% title('Vehicle 2')
% 
% subplot(3,1,3)
% plot(gospaLocal3)
% hold on
% plot(gospaFused3)
% legend({'Local Tracks','Fused Tracks'},'Location','southwest')
% ylim([min([gospaLocal1, gospaFused1, ...
%            gospaLocal2, gospaFused2, ...
%            gospaLocal3, gospaFused3])-2,...
%       max([gospaLocal1, gospaFused1, ...
%            gospaLocal2, gospaFused2, ...
%            gospaLocal3, gospaFused3])+2])
% title('Vehicle 3')




% gospaFigure=figure;
% figure(gospaFigure);
% subplot(3,1,1)
% plot(gospaLocal1)
% hold on
% plot(gospaFused1)
% legend({'Local Tracks','Fused Tracks'},'Location','southwest')
% ylim([min([gospaLocal1, gospaFused1, ...
%            gospaLocal2, gospaFused2, ...
%            gospaLocal3, gospaFused3])-2,...
%       max([gospaLocal1, gospaFused1, ...
%            gospaLocal2, gospaFused2, ...
%            gospaLocal3, gospaFused3])+2])
% title('Vehicle 1')
% 
% subplot(3,1,2)
% plot(gospaLocal2)
% hold on
% plot(gospaFused2)
% legend({'Local Tracks','Fused Tracks'},'Location','southwest')
% %ylim([min([gospaLocal2, gospaFused2])-2,max([gospaLocal2, gospaFused2])+2])
% ylim([min([gospaLocal1, gospaFused1, ...
%            gospaLocal2, gospaFused2, ...
%            gospaLocal3, gospaFused3])-2,...
%       max([gospaLocal1, gospaFused1, ...
%            gospaLocal2, gospaFused2, ...
%            gospaLocal3, gospaFused3])+2])
% title('Vehicle 2')
% 
% subplot(3,1,3)
% plot(gospaLocal3)
% hold on
% plot(gospaFused3)
% legend({'Local Tracks','Fused Tracks'},'Location','southwest')
% ylim([min([gospaLocal1, gospaFused1, ...
%            gospaLocal2, gospaFused2, ...
%            gospaLocal3, gospaFused3])-2,...
%       max([gospaLocal1, gospaFused1, ...
%            gospaLocal2, gospaFused2, ...
%            gospaLocal3, gospaFused3])+2])
% title('Vehicle 3')
% sgtitle('Generalized optimal subpattern assignment (GOSPA) metric')
% 
