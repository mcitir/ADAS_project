function [allData, scenario, egoVehicle, sensor] = createIntersectionScenario
%intersection_map_MultiVehicle - Returns sensor detections
%    allData = intersection_map_MultiVehicle returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = intersection_map_MultiVehicle optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.12 (R2022a) and Automated Driving Toolbox 3.5 (R2022a).
% Generated on: 04-May-2022 15:10:45

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
sensor = createSensor(scenario);

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
    if sensor.HasRoadsInputPort
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
    display(scenario)
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

function sensor = createSensor(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensor = lidarPointCloudGenerator('SensorIndex', 1, ...
    'SensorLocation', [0.95 0], ...
    'ActorProfiles', profiles);

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario('GeographicReference', [52.51387 13.388605 0], ...
    'VerticalAxis', 'Y');

% Add all road segments
roadCenters = [-142.7736905867 -117.32847965146 -0.0026741778361128;
    -17.936618461067 -104.2777249004 -0.00087792435606815];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Taubenstra??e');

roadCenters = [-27.270993600869 -6.8991040152757 -6.1912066166681e-05;
    -26.666817426911 -13.431072733384 -6.9777420952377e-05;
    -26.232352837704 -18.160351514336 -7.9695372798838e-05;
    -26.123736485381 -19.328761562074 -8.2685261673987e-05;
    -24.915373746502 -31.869696005304 -0.00012821524564544;
    -20.23121334961 -80.675847385523 -0.00054243986708258;
    -20.00718600562 -83.02379472249 -0.00057187808312875;
    -18.968510788028 -93.862186470726 -0.00071905997533861;
    -18.907412004379 -94.474210642453 -0.00072791726965704;
    -18.472931404086 -98.847401500195 -0.00079294910860028;
    -17.936618461067 -104.2777249004 -0.00087792435606815];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '375935041-474362277-375935044-375935042-474362278');

roadCenters = [-151.86806842332 -22.197427652595 -0.0018428709187628;
    -143.08313410719 -19.593807995413 -0.0016316418774291;
    -110.86286888204 -15.911370813529 -0.00098131496502774;
    -27.270993600869 -6.8991040152757 -6.1912066166681e-05];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'J??gerstra??e');

roadCenters = [-142.7736905867 -117.32847965146 -0.0026741778361128;
    -150.96543378055 -31.656008251653 -0.0018614356535966;
    -151.27762290789 -28.362196754278 -0.0018533139694927;
    -151.86806842332 -22.197427652595 -0.0018428709187628];
laneSpecification = lanespec(1);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Glinkastra??e');

roadCenters = [-213.15615533283 -126.4951554667 -0.0048091489083788;
    -150.19419767717 -119.8431198127 -0.0028910141411345;
    -142.7736905867 -117.32847965146 -0.0026741778361128];
laneSpecification = lanespec(1);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Taubenstra??e');

roadCenters = [-17.936618461067 -104.2777249004 -0.00087792435606815;
    47.849025493646 -98.012624691356 -0.00093246947999148;
    58.894751359329 -96.966499890658 -0.0010087088530923;
    76.396817847012 -95.174697401552 -0.0011669430958534;
    189.91553601595 -83.576542125228 -0.0033692853704537];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Taubenstra??e');

roadCenters = [180.26486331472 11.631771075621 -0.0025526430437157;
    189.91553601595 -83.576542125228 -0.0033692853704537];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Charlottenstra??e');

roadCenters = [-27.270993600869 -6.8991040152757 -6.1912066166681e-05;
    9.3754568559694 -2.7374075722903 -7.4644130272361e-06];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'J??gerstra??e');

roadCenters = [9.3754568559694 -2.7374075722903 -7.4644130272361e-06;
    18.201010206153 -1.8137828613077 -2.617284516826e-05;
    39.742134813934 0.50090803123246 -0.00012357664111162;
    49.667487781981 0.24506122778239 -0.00019298073488233;
    59.721815301656 1.2466672694935 -0.00027913553144621;
    168.94781100847 11.809411323885 -0.0022438109592597;
    180.26486331472 11.631771075621 -0.0025526430437157];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'J??gerstra??e');

roadCenters = [170.05090548639 112.44846407901 -0.0032537541627136;
    171.03562742367 102.24438702096 -0.0031082256259367;
    171.61285302386 97.036638338154 -0.0030423093507252;
    171.94561622239 93.77623063001 -0.0030024641692492;
    180.26486331472 11.631771075621 -0.0025526430437157];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Charlottenstra??e');

roadCenters = [9.3754568559694 -2.7374075722903 -7.4644130272361e-06;
    8.0855484791744 8.9689406129208 -1.1423739334049e-05;
    6.4629909888209 21.632271950116 -3.9966785911005e-05;
    0.69245655318744 69.537060607516 -0.00037924323582672;
    -0.25118467553048 80.164024382325 -0.00050397100068977;
    -0.35980498832576 81.354689455843 -0.00051905712842171;
    -1.6768223018323 95.15305140866 -0.00071026725131418];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '812557613-812557616-812557614-812557615-812557617');

roadCenters = [39.069246970044 99.392864778174 -0.00089414054392023;
    50.854502048105 100.56138255466 -0.00099536649189247;
    53.895857419527 100.85073554343 -0.0010248589161819;
    73.352377200418 102.67593272526 -0.0012476667230388;
    88.776379423606 103.98925772814 -0.0014645756011404;
    153.5407670639 110.73425157277 -0.0028058184247541;
    158.17066731048 111.21289037092 -0.0029270465032738;
    170.05090548639 112.44846407901 -0.0032537541627136];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Franz??sische Stra??e');

roadCenters = [-1.6768223018323 95.15305140866 -0.00071026725131418;
    24.548121915498 97.846018691546 -0.00079794660606325;
    29.714357695629 98.402432845666 -0.00082844106350421;
    39.069246970044 99.392864778174 -0.00089414054392023];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Franz??sische Stra??e');

roadCenters = [170.05090548639 112.44846407901 -0.0032537541627136;
    180.01672520104 113.43918625261 -0.003544217388928;
    184.66698691199 113.90672311552 -0.0036852166549224;
    288.67648275498 124.27164434784 -0.0077301273674237;
    300.58381396479 125.46302519274 -0.0083023443996737];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Franz??sische Stra??e');

roadCenters = [170.05090548639 112.44846407901 -0.0032537541627136;
    168.6859881406 123.56500074997 -0.0034233400003671;
    168.20385306404 127.53757724896 -0.0034888621340485;
    161.89544404569 178.94739492606 -0.0045616161500561;
    158.20145301164 209.05886523291 -0.0053853619980657;
    154.2094015306 220.3978777505 -0.0056696887104692;
    148.55294619204 272.11931640915 -0.0075334342801341];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Charlottenstra??e');

roadCenters = [-162.66591459691 77.863287183281 -0.0025453655764949;
    -150.99596647595 79.10921743588 -0.002274357939271;
    -147.77128725022 79.454078202586 -0.0022032796752569;
    -1.6768223018323 95.15305140866 -0.00071026725131418];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Franz??sische Stra??e');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-129.5 79.8 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [-129.5 79.8 0;
    -99.9 82.6 0;
    -64.1 86.2 0;
    -42.6 88.9 0;
    -4.5 93.1 0;
    23.8 96 0;
    47.29 98.57 0.01;
    65.71 100.32 0.01;
    83.31 101.83 0.01;
    104.36 103.66 0.01;
    161.67 109.84 0.01;
    180.28 111.66 0.01;
    307.3 124.5 0];
speed = [20;20;20;20;20;20;20;20;20;20;20;20;20];
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-90.6971890858616 83.7168064104334 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [-90.6971890858616 83.7168064104334 0;
    -38.4 89.5 0;
    -12.6 92 0;
    5.9 93.9 0;
    41.5 97.6 0;
    71.9 100.6 0;
    90.8 102.1 0;
    114.7 104.4 0;
    139.7 107 0;
    166.1 110 0;
    196.28 113.18 0.01;
    216.83 115.22 0.01;
    281.3 121.4 0;
    304.1 124 0];
speed = [20;20;20;20;20;20;20;20;20;20;20;20;20;20];
trajectory(car1, waypoints, speed);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-110.2 85.4 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
waypoints = [-110.2 85.4 0;
    -80.3 88.3 0;
    -47 91.9 0;
    -19.6 94.8 0;
    6.8 97.8 0;
    37.7 101.2 0;
    71.3 104 0;
    92.3 106.34 0.01;
    129.76 110.05 0.01;
    217.4 118.9 0;
    267.2 123.7 0;
    306.6 128.1 0];
speed = [30;30;30;30;30;30;30;30;30;30;30;30];
trajectory(car2, waypoints, speed);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-92.7 87.1 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');
waypoints = [-92.7 87.1 0;
    -69.2 89.7 0;
    -36.5 93.1 0;
    1.5 97.1 0;
    53.1 102.5 0;
    114.7 108.2 0;
    150.1 111.9 0;
    198.5 116.7 0;
    246.4 121.6 0;
    310.6 128.5 0];
speed = [32;32;32;32;32;32;32;32;32;32];
trajectory(car3, waypoints, speed);

car4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-134.8 83.2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car4');
waypoints = [-134.8 83.2 0;
    -116.9 84.5 0;
    -97.1 86.7 0;
    -58.2 90.8 0;
    17.7 98.7 0;
    61.7 103.3 0;
    124 109.2 0;
    168.7 113.7 0;
    325.9 130.2 0];
speed = [28;28;28;28;28;28;28;28;28];
trajectory(car4, waypoints, speed);

car5 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-109.4 81.8 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car5');
waypoints = [-109.4 81.8 0;
    -76.7 85.1 0;
    -53.1 87.9 0;
    13.5 94.7 0;
    39 97.5 0;
    144.3 107.8 0;
    189.2 112.5 0;
    228.2 116 0;
    258.2 119.6 0;
    313.4 125.7 0];
speed = [20;22;24;20;18;17;20;20;20;20];
waittime = [0;0;0;0;0;0;0;0;0;0];
trajectory(car5, waypoints, speed, waittime);

car6 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-148.807353030917 81.4844779276272 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car6');
waypoints = [-148.807353030917 81.4844779276272 0.01;
    -122.6 84.4 0;
    -83.7 87.9 0;
    -26.7 94.2 0;
    29.6 100.4 0;
    80.7 105.2 0;
    136.9 110.7 0;
    187.4 115.9 0;
    315.1 129.2 0];
speed = [23;23;23;23;23;23;23;23;23];
trajectory(car6, waypoints, speed);

car7 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-172.07 78.92 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car7');
waypoints = [-172.07 78.92 0;
    -153.26 80.74 0.01;
    -125.61 83.69 0.01;
    -31.7 93.42 0.01;
    41.3 101.5 0;
    160.1 113.1 0;
    321.1 129.8 0];
speed = [22;25;25;22;22;22;22];
waittime = [0;0;0;0;0;0;0];
trajectory(car7, waypoints, speed, waittime);

