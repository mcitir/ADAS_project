function [allData, scenario, sensors, numSensors, attachedVehicle] = straight()
%Straight - Returns sensor detections
%    allData = Straight returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = Straight optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.12 (R2022a) and Automated Driving Toolbox 3.5 (R2022a).
% Generated on: 05-May-2022 15:14:41

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
[sensors, numSensors, attachedVehicle] = createSensor(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {}, 'PointClouds', {}, 'INSMeasurements', {});
running = true;
while running

    % Generate the target poses of all actors relative to the ego vehicle
    poses = targetPoses(egoVehicle);
    time  = scenario.SimulationTime;

    % Generate detections for the sensor
    laneDetections = [];
    objectDetections = [];
    insMeas = [];
    if sensors.HasRoadsInputPort
        rdmesh = roadMesh(egoVehicle,min(500,sensor.MaxRange));
        [ptClouds, isValidPointCloudTime] = sensor(poses, rdmesh, time);
    else
        [ptClouds, isValidPointCloudTime] = sensor(poses, time);
    end

    % Aggregate all detections into a structure for later use
    if isValidPointCloudTime
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections}, ...
            'LaneDetections', {laneDetections}, ...
            'PointClouds',   {ptClouds}, ... %#ok<AGROW>
            'INSMeasurements',   {insMeas}); %#ok<AGROW>
    end

    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release the sensor object so it can be used again.
release(sensor);

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function [sensors,numSensors,attachedVehicle] = createSensor(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);

% Vehicle 1
sensors{1} = lidarPointCloudGenerator('SensorIndex', 1, ...
    'SensorLocation', [0.95 0], ...
    'MaxRange', 150, ...
    'HasNoise', false, ...
    'ActorProfiles', profiles);

% Vehicle 2
sensors{2} = lidarPointCloudGenerator('SensorIndex', 2, ...
    'SensorLocation', [0.95 0], ...
    'MaxRange', 150, ...
    'HasNoise', false, ...
    'ActorProfiles', profiles);
attachedVehicle = [1;2];
numSensors = numel(sensors);

function [scenario, egoVehicle, car1] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [30 150 0;
    30 -650 0];
laneSpecification = lanespec(3, 'Width', 6);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [30 140 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [30 140 0;
    30 -250 0;
    30 -650 0];
speed = [15;15;15];
waittime = [0;0;0];
trajectory(egoVehicle, waypoints, speed, waittime);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [24 115 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [24 115 0;
    24 -150 0;
    24 -320 0;
    24 -650 0];
speed = [14;17;15;14];
waittime = [0;0;0;0];
trajectory(car1, waypoints, speed, waittime);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [36 130 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
waypoints = [36 130 0;
    36 -100 0;
    36 -250 0;
    36 -450 0;
    36 -650 0];
speed = [15;15;15;15;15];
waittime = [0;0;0;0;0];
trajectory(car2, waypoints, speed, waittime);

