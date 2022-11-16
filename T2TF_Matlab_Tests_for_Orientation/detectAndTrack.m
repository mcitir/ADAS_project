function [tracks,manipulatedTracks,wasTrackerUpdated,detections, manipulatedDetections] = detectAndTrack(agent,time)

poses = targetPoses(agent.Actor);
[detections,manipulatedDetections, isValid] = vehicleDetections(agent.Actor, agent.Sensors,poses,time, agent.Manipulated);
%manipulatedDetections = manipulate(detections, "detections", agent.Manipulated);
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

