function [scenario, egoVehicle, secondVehicle] = createDrivingScenario
% Construct a drivingScenario object
scenario = drivingScenario('SampleTime', 0.1);


% Add all road segments
roadCenters = [30 150 0;
    30 -650 0];
laneSpecification = lanespec(3, 'Width', 6);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% % Add all road segments
% roadCenters = [50.8 0.5 0; 253.4 1.5 0];
% roadWidth = 12;
% road(scenario, roadCenters, roadWidth);
% 
% roadCenters = [100.7 -100.6 0; 100.7 103.7 0];
% road(scenario, roadCenters);
% 
% roadCenters = [201.1 -99.2 0; 199.7 99.5 0];
% road(scenario, roadCenters);

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [30 140 0], ...
    'PlotColor', [0 0.7410 0.4470],...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [30 140 0;
    30 -250 0;
    30 -650 0];
speed = [15;15;15];
waittime = [0;0;0];
trajectory(egoVehicle, waypoints, speed, waittime);

% % Add the ego vehicle
% egoVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [65.1 -0.9 0], 'PlotColor', [0 0.7410 0.4470]);
% waypoints = [71 -0.5 0; 148.7 -0.5 0];
% speed = 12;
% trajectory(egoVehicle, waypoints, speed);

% % Add the second vehicle
secondVehicle = vehicle(scenario, ...
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
trajectory(secondVehicle, waypoints, speed, waittime);

% % Add the second vehicle
% secondVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [55.1 -0.9 0]);
% waypoints = [61 -0.5 0; 138.7 -0.5 0];
% speed = 12;
% trajectory(secondVehicle, waypoints, speed);

% Add the parked cars
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

% vehicle(scenario, 'ClassID', 1, 'Position', [111.0 -3.6 0]);
% vehicle(scenario, 'ClassID', 1, 'Position', [140.6 -3.6 0]);
% vehicle(scenario, 'ClassID', 1, 'Position', [182.6 -3.6 0]);
% vehicle(scenario, 'ClassID', 1, 'Position', [211.3 -4.1 0]);

% % Add pedestrian
% actor(scenario, 'ClassID', 4, 'Length', 0.5, 'Width', 0.5, ...
%     'Height', 1.7, 'Position', [130.3 -2.7 0], 'RCSPattern', [-8 -8;-8 -8]);
% 
% % Add parked truck
% vehicle(scenario, 'ClassID', 2, 'Length', 8.2, 'Width', 2.5, ...
%     'Height', 3.5, 'Position', [117.5 -3.5 0]);
end