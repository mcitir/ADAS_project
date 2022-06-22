function [tracks,wasTrackerUpdated] = vehicleTracks(actor, sensors, poses, time, plotter)
% Create detections from the vehicle
numSensors = numel(sensors);
tracks = objectTrack.empty;
isValidTime = false(1, numSensors);

% Generate detections for each sensor
for sensorIndex = 1:numSensors
    sensor = sensors{sensorIndex};
    if isa(sensor, 'radarDataGenerator') && strcmpi(sensor.TargetReportFormat,'Tracks')
        [sensorTracks, ~, sensorConfig] = sensor(poses, time);
        if islogical(sensorConfig)
            isValidTime(sensorIndex) = sensorConfig;
        else
            isValidTime(sensorIndex) = sensorConfig.IsValidTime;
        end
        numObjects = numel(sensorTracks);
        tracks = [tracks; sensorTracks(1:numObjects)]; %#ok<AGROW>
    end
end
wasTrackerUpdated = any(isValidTime);

if ~wasTrackerUpdated % No vehicle tracking sensor udpated
    return
end

% Add vehicle position and velocity to track state parameters
for i = 1:numel(tracks)
    tracks(i).StateParameters.OriginPosition = tracks(i).StateParameters.OriginPosition + actor.Position';
    tracks(i).StateParameters.OriginVelocity = tracks(i).StateParameters.OriginVelocity + actor.Velocity';
end

% Plot tracks
if numel(tracks)>0
    trPos = arrayfun(@(t)t.State([1,3]), tracks, 'UniformOutput', false);
    trPos = cell2mat(trPos')' + actor.Position(1:2);
    plotTrack(plotter, trPos);
end
end