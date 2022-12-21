function [tracks,manipulatedTracks,wasTrackerUpdated,detections, manipulatedDetections] = detectAndTrack(agent,time)

poses = targetPoses(agent.Actor);
% [detections,manipulatedDetections, isValid] = vehicleDetections(agent.Actor, agent.Sensors,poses,time, agent.Manipulated);
[detections,manipulatedDetections, isValid] = vehicleDetections(agent.Actor, agent.Sensors,poses,time, []);

%manipulatedDetections = manipulate(detections, "detections", agent.Manipulated);
if sum(agent.Manipulated) ~= 0 && size(detections,1) ~= 0
    detections{end+1,1} = detections{1,1};
    % detections{end}.Measurement = [25; -3; 0];
    detections{end,1}.MeasurementNoise = 1.*detections{1,1}.MeasurementNoise;
    detections{end}.Measurement = [25; 5; 0];
    detections{end+1, 1} = detections{end};
    detections{end+1, 1} = detections{end};
    detections{end+1, 1} = detections{end};
end
% Find tracks from detections
if isValid
    % StateParameters is empty by default
    agent.Tracker.StateParameters = struct(...
        'Frame','Rectangular', ...
        'OriginPosition', [0,0,0], ...
        'OriginVelocity', [0,0,0]);
    tracks = agent.Tracker(detections,time); % Tracks from detection
    
    manipulatedTracks = agent.ManipulatedTracker(manipulatedDetections,time); % Tracks from detection
    wasTrackerUpdated = true;
else
    tracks = objectTrack.empty(0,1);
    manipulatedTracks = objectTrack.empty(0,1);
    wasTrackerUpdated = false;

end

% Get additional tracks from tracking sensors
[sensorTracks,wasSensorTrackerUpdated] = vehicleTracks(agent.Sensors,poses,time);
tracks = vertcat(tracks,sensorTracks);
manipulatedTracks = vertcat(manipulatedTracks,sensorTracks); % No manipulation on sensor tracks
wasTrackerUpdated = wasTrackerUpdated || wasSensorTrackerUpdated;

end

