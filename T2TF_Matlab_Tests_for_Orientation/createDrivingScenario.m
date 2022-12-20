function [scenario, egoVehicle, secondVehicle, thirdVehicle] = createDrivingScenario
% Construct a drivingScenario object for standing traffic case
scenario = drivingScenario('SampleTime', 0.1);

% Add all road segments
roadCenters = [0 0 0; 100 0 0]; 
roadWidth = 12;
road(scenario, roadCenters, roadWidth);

roadCenters = [50 -50 0; 50 50 0];
road(scenario, roadCenters, roadWidth);


% Add the ego vehicle
egoVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [25 -3 0], 'PlotColor', [0 0.7410 0.4470]);

% Add the second vehicle
secondVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [47 25 0],'Yaw', -90);

% Add the third vehicle
thirdVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [70 3 0],'Yaw', 180);

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
    98.7 -3.5 0];
speed = 10;
trajectory(dynamicVehicle,waypoints,speed)

% Add the parked cars
vehicle(scenario, 'ClassID', 1, 'Position', [48.3 -0.8 0]);

end