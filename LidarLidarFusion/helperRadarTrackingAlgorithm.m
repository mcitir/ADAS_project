classdef helperRadarTrackingAlgorithm < matlab.System
    % This class defines 1 tracker for all radar sensor and uses GM-PHD
    % Rectangular object tracker to track the targets. The output of the
    % tracker is a list of concatenated tracks for each object.
    
    %#codegen
    
    properties
        Tracker
    end
    
    properties (Access = protected)
        SensorResolutions
    end
    
    methods
        function obj = helperRadarTrackingAlgorithm(radars,varargin)
            setProperties(obj,nargin-1,varargin{:});
            setupTrackers(obj,radars);
        end
    end
    
    methods (Access = protected)
        function tracksOut = stepImpl(obj, egoPose, detections, time)
            tracker = obj.Tracker;
            detections = preprocessDetections(detections, egoPose, obj.SensorResolutions);
            configs = tracker.SensorConfigurations;
            for i = 1:numel(configs)
                configs{i}.SensorTransformParameters(2).OriginPosition = egoPose.Position(:);
                configs{i}.SensorTransformParameters(2).OriginVelocity = egoPose.Velocity(:);
                configs{i}.SensorTransformParameters(2).Orientation = rotmat(quaternion([egoPose.Yaw egoPose.Pitch egoPose.Roll],'eulerd','ZYX','frame'),'frame');
            end
            tracks = tracker(detections, configs, time);
            
            % Convert to objectTrack format for fusion in MATLAB
            if coder.target('MATLAB')
                tracksOut = repmat(objectTrack,numel(tracks),1);
                for i = 1:numel(tracks)
                    tracksOut(i) = objectTrack(tracks(i));
                end
            else % In codegen convert to structs with similar fields as lidar
                tracksOut = tracks;
            end
        end
    end
    
    methods (Access = protected)
        function setupTrackers(obj, radars)
            % Setup a GM-PHD Tracker for each radar
            numSensors = numel(radars);
            
            configurations = cell(numSensors, 1);
            
            partFcn = @(x)partitionDetections(x,3,5);
            
            for i = 1:numSensors
                sensorConfig = createSensorConfiguration(radars{i});
                configurations{i} = sensorConfig;
            end

            tracker = trackerPHD('TrackerIndex',1,...
                'HasSensorConfigurationsInput',true,...
                'SensorConfigurations',configurations,...
                'PartitioningFcn',partFcn,...
                'BirthRate',1e-3,...
                'AssignmentThreshold',320,...% Minimum negative log-likelihood of a detection cell to add birth components
                'ExtractionThreshold',0.85,...% Weight threshold of a filter component to be declared a track
                'ConfirmationThreshold',0.98,...% Weight threshold of a filter component to be declared a confirmed track
                'MergingThreshold',50,...% Threshold to merge components
                'DeletionThreshold',1e-2,...% Threshold to delete components
                'LabelingThresholds',[1.1 0.01 0]...% This translates to no track-splitting. Read LabelingThresholds help
                );
            
            obj.Tracker = tracker;
            
            obj.SensorResolutions = repmat(struct('SensorIndex',1,...
                'SensorResolution',zeros(2,1)),numSensors,1);
            
            for i = 1:numSensors
                obj.SensorResolutions(i).SensorIndex = configurations{i}.SensorIndex;
                obj.SensorResolutions(i).SensorResolution = configurations{i}.SensorResolution;
            end
        end
    end
end

function partitions = truePartition(detections)
% For tuning, use this partition function with trackerPHD to discern if
% tracking performance is poor because of filtering or because of
% clustering. This returns true clustering information.
tgtIdx = cellfun(@(x)x.ObjectAttributes{1}.TargetIndex,detections);
tgtIdx(tgtIdx == -1) = max(tgtIdx) + (1:sum(tgtIdx == -1));

uqTgtIdx = unique(tgtIdx);
partitions = zeros(numel(detections),1);
for i = 1:numel(uqTgtIdx)
    partitions(tgtIdx == uqTgtIdx(i)) = i;
end
end

function partitions = partitionWithoutDoppler(detections,varargin)
    for i = 1:numel(detections)
        detections{i}.Measurement(3) = 0;
        detections{i}.MeasurementNoise(3,3) = 1;
    end
    partitions = partitionDetections(detections,varargin{:});
end

function detsOut = preprocessDetections(dets,egoPose,resolutions)
% Detections cannot report an uncertainty better than sensor's resolution
% and must be updated to include ego vehicle or INS information

% Update Measurement paramters
n = numel(dets);
detsOut = cell(n,1);
for i = 1:n
    st = dets{i}.MeasurementParameters;
    st.Frame = drivingCoordinateFrameType(1);
    st.OriginVelocity = zeros(3,1);
    if coder.target('MATLAB')
        thisDet = dets{i};
        thisDet.MeasurementParameters = [st;st];
    else
        thisDet = objectDetection(dets{i}.Time,dets{i}.Measurement,...
            'MeasurementNoise',dets{i}.MeasurementNoise,...
            'SensorIndex',dets{i}.SensorIndex,...
            'MeasurementParameters',{[st;st]},...
            'ObjectClassID',dets{i}.ObjectClassID,...
            'ObjectAttributes',dets{i}.ObjectAttributes);
    end
    
    idx = thisDet.SensorIndex;
    res = resolutions(idx).SensorResolution;
    resNoise = diag(res.^2/4);
    thisDet.MeasurementParameters(1).Frame(:) = drivingCoordinateFrameType(2);
    thisDet.MeasurementParameters(2).Frame(:) = drivingCoordinateFrameType(1);
    thisDet.MeasurementParameters(2).OriginPosition = egoPose.Position(:);
    thisDet.MeasurementParameters(2).OriginVelocity = egoPose.Velocity(:);
    thisDet.MeasurementParameters(2).Orientation = rotmat(quaternion([egoPose.Yaw egoPose.Pitch egoPose.Roll],'eulerd','ZYX','frame'),'frame')';
    thisDet.MeasurementNoise(1:2,1:2) = resNoise;
    thisDet.MeasurementNoise(3,3) = 0.5^2/4;
    detsOut{i} = thisDet;
end

end

function filter = helperInitRectangularFilter(varargin)
% helperInitRectangularFilter A function to initialize the rectangular
% target PHD filter for the Extended Object Tracking example. Read
% helperInitRectangularFilterMultiModel for description on when this
% type of approach can fail.

% Copyright 2019 The MathWorks, Inc.

if nargin == 0
    % If called with no inputs, simply use the initctrectgmphd function to
    % create a PHD filter with no components.
    filter = initctrectgmphd;
    % Set process noise
    filter.ProcessNoise = diag([5^2/12 5^2/12]);
else
    detections = varargin{1};
    
    % Create a GM-PHD filter with rectangular model
    filter = initctrectgmphd(detections);
    filter.States(6:7,1) = [6;3];
    filter.StateCovariances(4,4,1) = 45^2/12;
    filter.StateCovariances(5,5,1) = 30^2/12;
    lCov = 1;
    wCov = 1;
    lwCorr = 0.5;
    lwCov = sqrt(lCov*wCov)*lwCorr;
    filter.StateCovariances(6:7,6:7,1) = [lCov lwCov;lwCov wCov];
end

end

function filter = helperInitRectangularFilterMultiModel(varargin)
% helperInitRectangularFilterMultiModel A function to initialize the
% rectangular target PHD filter for the Extended Object Tracking using
% Radar.

% Copyright 2019 The MathWorks, Inc.

if nargin == 0
    % If called with no inputs, simply use the initctrectgmphd function to
    % create a PHD filter with no components.
    filter = initctrectgmphd;
    filter.MaxNumComponents = 10000;
    % Set process noise
    filter.ProcessNoise = diag([1 3]);
else
    % When called with detections input, add two components to the filter,
    % one for car and one for truck, More components can be added based on
    % prior knowledge of the scenario, example, pedestrian or motorcycle.
    % This is a "multi-model" type approach. Another approach can be to add
    % only 1 component with a higher covariance in the dimensions. The
    % later is computationally less demanding, but has a tendency to track
    % observable dimensions of the object. For example, if only the back is
    % visible, the measurement noise or occlusions may cause the length of
    % the object to shrink. The other approach is included as
    % helperInitRectangularFilter in this class. Note that the other
    % approach may have issues when clustering is ambiguous or if the track
    % is partially observable by a sensor.
    
    % Detections
    detections = varargin{1};
    
    % Create a GM-PHD filter with rectangular model
    filter = initctrectgmphd(detections);
    
    % Length width of a passenger car
    filter.States(6:7,1) = [4.7;1.8];
    
    % High certainty in dimensions
    lCov = 1e-4;
    wCov = 1e-4;
    lwCorr = 0.5;
    lwCov = sqrt(lCov*wCov)*lwCorr;
    filter.StateCovariances(6:7,6:7,1) = [lCov lwCov;lwCov wCov];
    
    % Add one more component by appending the filter with itself.
    append(filter,filter);
    
    % Set length and width to a truck dimensions
    filter.States(6:7,2) = [8.1;2.45];
    
    % Relative weights of each component
    filter.Weights = [0.7 0.3];
end

end

function config = createSensorConfiguration(radar)
assert(isa(radar,'drivingRadarDataGenerator'));
fov = radar.FieldOfView;
sensorLimits = [-fov(1)/2 fov(1)/2;0 inf]; % Kill tracks outside max-range by making them detectable
sensorResolution = [radar.AzimuthResolution;radar.RangeResolution];
Kc = radar.FalseAlarmRate/(radar.AzimuthResolution*radar.RangeResolution*radar.RangeRateResolution);
Pd = 0.9; % Probability of generating at least 1 detection

sensorPos = radar.MountingLocation(:);
sensorOrient = rotmat(quaternion(radar.MountingAngles, 'eulerd', 'ZYX', 'frame'),'frame');

sensorTransformParameters = struct('Frame',drivingCoordinateFrameType(2),...
    'OriginPosition', sensorPos,...
    'OriginVelocity', zeros(3,1),...% Sensor does not move relative to ego
    'Orientation', sensorOrient,...
    'IsParentToChild',true,...% Frame rotation is supplied as orientation
    'HasElevation',false,...
    'HasVelocity',false); % Does not use elevation or rr to measure detectability.

sensorTransformParameters = [sensorTransformParameters;sensorTransformParameters];

sensorTransformParameters(2).OriginPosition = zeros(3,1);
sensorTransformParameters(2).Frame(:) = drivingCoordinateFrameType(1);

config = trackingSensorConfiguration(radar.SensorIndex,...
    'IsValidTime', true,... % Only updated when detections are available, so always true
    'SensorLimits',sensorLimits,...
    'SensorResolution', sensorResolution,...
    'DetectionProbability',Pd,...
    'ClutterDensity', Kc,...
    'SensorTransformFcn',@ctrectcorners,...
    'SensorTransformParameters', sensorTransformParameters);
config.FilterInitializationFcn = @helperInitRectangularFilterMultiModel;

config.MinDetectionProbability = 0.05;

end

