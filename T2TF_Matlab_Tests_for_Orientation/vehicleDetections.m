function [objectDetections, manipulatedObjectDetections, isValid] = vehicleDetections(actor, sensors, poses, time, manipulatedSensors)
numSensors = numel(sensors);
objectDetections = {};
manipulatedObjectDetections = {};
%manipulatedObjectDets = {};
isValidTime = false(1, numSensors);

% Generate detections for each sensor
for sensorIndex = 1:numSensors
    sensor = sensors{sensorIndex};
    % Check is a camera or a radar which does not generate detection as tracks 
    if isa(sensor, 'visionDetectionGenerator') || isa(sensor, 'radarDataGenerator')  || ~strcmpi(sensor.TargetReportFormat,'Tracks')
%         if ~strcmpi(sensor.TargetReportFormat,'Tracks')
        % Generates sensor detections for actors except ego vehicle
        [objectDets, ~, sensorConfig] = sensor(poses, time); 

        if islogical(sensorConfig)
            isValidTime(sensorIndex) = sensorConfig;
        else
            isValidTime(sensorIndex) = sensorConfig.IsValidTime;
        end
        if ~iscell(objectDets)
            for dets=1:numel(objectDets)
                objectDetsTemp{dets,1} = objectDets(dets);%#ok<AGROW>
                objectDets = cellfun(@(d) setAtt(d,actor), objectDetsTemp, 'UniformOutput', false);
                numObjects = numel(objectDets);
                
                if ismember(sensorIndex, manipulatedSensors)
                    manipulatedObjectDets = manipulate(objectDets); % Manipulation
                    objectDetections = [objectDetections; manipulatedObjectDets(1:numObjects)]; %#ok<AGROW>
                else
                    
                    objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
                end
            end
        else
                objectDets = cellfun(@(d) setAtt(d,actor), objectDets, 'UniformOutput', false);
                numObjects = numel(objectDets);

                if ismember(sensorIndex, manipulatedSensors)
                    manipulatedObjectDets = manipulate(objectDets); % Manipulation
                    manipulatedObjectDetections = [manipulatedObjectDetections; manipulatedObjectDets(1:numObjects)]; %#ok<AGROW>
                    objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
                else
                    manipulatedObjectDetections = [manipulatedObjectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
                    objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
                end

        end
        
    end
end
isValid = any(isValidTime);

end

function d = setAtt(d, actor)
    % Set the attributes to be a structure
    d.ObjectAttributes = struct;
    % Keep only the position measurement and remove velocity
    if ~isstruct(d.MeasurementParameters)
        d.Measurement = d.Measurement(1:3);
        d.MeasurementNoise = d.MeasurementNoise(1:3,1:3);
    
        d.MeasurementParameters{1}.OriginPosition = actor.Position;
        d.MeasurementParameters{1}.OriginVelocity = actor.Velocity;
    
        rotAngle2 = [actor.Yaw actor.Pitch actor.Roll]; % [yaw,pitch,row] scenario ya gore
        rotQuat2 = quaternion(rotAngle2,'Eulerd','ZYX','frame');
        rotMatrix2 = rotmat(rotQuat2,'frame');
    
        d.MeasurementParameters{1}.Orientation =rotMatrix2;
        d.MeasurementParameters{1}.IsParentToChild = true;
        d.MeasurementParameters{1}.HasElevation = false;
        d.MeasurementParameters{1}.HasAzimuth = false;
        %d.MeasurementParameters{1}.HasRange = true;
        d.MeasurementParameters{1}.HasVelocity = false;
    else
        d.MeasurementParameters.OriginPosition = actor.Position;
        d.MeasurementParameters.OriginVelocity = actor.Velocity;
    
        rotAngle2 = [actor.Yaw actor.Pitch actor.Roll]; % [yaw,pitch,row] scenario ya gore
        rotQuat2 = quaternion(rotAngle2,'Eulerd','ZYX','frame');
        rotMatrix2 = rotmat(rotQuat2,'frame');
    
        d.MeasurementParameters.Orientation =rotMatrix2;
        d.MeasurementParameters.IsParentToChild = true;
        d.MeasurementParameters.HasElevation = false;
        d.MeasurementParameters.HasAzimuth = false;
        d.MeasurementParameters.HasRange = true;
        %d.MeasurementParameters.HasRangeRate = true;
        d.MeasurementParameters.HasVelocity = false;
    end
end