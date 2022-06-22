function [scenario, egoVehicle, secondVehicle, thirdVehicle] = createDrivingScenario
% Construct a drivingScenario object for standing traffic case

scenario = drivingScenario('SampleTime', 0.1);

% Add all road segments
roadCenters = [0 0 0; 100 0 0]; %[50.8 0.5 0; 253.4 1.5 0];
roadWidth = 12;
road(scenario, roadCenters, roadWidth);

roadCenters = [50 -50 0; 50 50 0]; %[100.7 -100.6 0; 100.7 103.7 0];
road(scenario, roadCenters, roadWidth);

%roadCenters = [201.1 -99.2 0; 199.7 99.5 0];
%road(scenario, roadCenters);

% Add the ego vehicle
egoVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [36 -3 0], 'PlotColor', [0 0.7410 0.4470]);
%waypoints = [36 -3 0; 36.1 -3 0];
%speed = 0.01;
%trajectory(egoVehicle, waypoints, speed);

% Add the second vehicle
secondVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [47 12 0],'Yaw', -90);
%waypoints = [47 12 0; 47 12 0];
%speed = 0;
%trajectory(secondVehicle, waypoints, speed);

% Add the third vehicle
thirdVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [62.4 3 0],'Yaw', 180);
%waypoints = [51 -0.5 0; 128.7 -0.5 0];
%speed = 12;
%trajectory(thirdVehicle, waypoints, speed);

% Add a dynamic vehicle
dynamicVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [54 -50 0]);
waypoints = [54.3 -46.38 0;
    54.1 -36.4 0;
    53.8 -25.6 0;
    53.6 -12.4 0;
    54.5 -5 0;
    63.4 -3.5 0;
    70 -3.4 0;
    78.9 -3.5 0;
    88.2 -3.4 0;
    98.7 -3.5 0];  %[54 -50 0; 54 50 0];
speed = 10;
trajectory(dynamicVehicle,waypoints,speed)

% Add the parked cars
vehicle(scenario, 'ClassID', 1, 'Position', [48.3 -0.8 0]);

%{
vehicle(scenario, 'ClassID', 1, 'Position', [140.6 -3.6 0]);
vehicle(scenario, 'ClassID', 1, 'Position', [182.6 -3.6 0]);
vehicle(scenario, 'ClassID', 1, 'Position', [211.3 -4.1 0]);

% Add pedestrian
actor(scenario, 'ClassID', 4, 'Length', 0.5, 'Width', 0.5, ...
    'Height', 1.7, 'Position', [130.3 -2.7 0], 'RCSPattern', [-8 -8;-8 -8]);

% Add parked truck
vehicle(scenario, 'ClassID', 2, 'Length', 8.2, 'Width', 2.5, ...
    'Height', 3.5, 'Position', [117.5 -3.5 0]);
end
%}