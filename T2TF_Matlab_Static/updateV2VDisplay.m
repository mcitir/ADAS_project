function updateV2VDisplay(plotters, scenario, sensors, attachedVehicle)
% Updates the display of the road and actors on the road

% get the road boundaries and rectangular outlines
rb = roadBoundaries(scenario);
[position, yaw, length, width, originOffset, color] = targetOutlines(scenario.Actors(1));
position = position + scenario.Actors(1).Position(1:2); % Calculates global pos of actors

% update the bird's-eye plotters with the road and actors
plotLaneBoundary(plotters.lb1Plotter, rb);
plotOutline(plotters.ol1Plotter, position, yaw, length, width, ...
    'OriginOffset', originOffset, 'Color', color);
plotLaneBoundary(plotters.lb2Plotter, rb);
plotOutline(plotters.ol2Plotter, position, yaw, length, width, ...
    'OriginOffset', originOffset, 'Color', color);
plotLaneBoundary(plotters.lb3Plotter, rb);
plotOutline(plotters.ol3Plotter, position, yaw, length, width, ...
    'OriginOffset', originOffset, 'Color', color);

for i = 1:numel(sensors)
    shift = scenario.Actors(attachedVehicle(i)).Position(1:2);
    if isa(sensors{i},'radarDataGenerator') || isa(sensors{i},'drivingRadarDataGenerator')
        plotCoverageArea(plotters.SensorPlotter(i), sensors{i}.MountingLocation(1:2) + shift,...
            sensors{i}.RangeLimits(2), sensors{i}.MountingAngles(3)+scenario.Actors(attachedVehicle(i)).Yaw,...
            sensors{i}.FieldOfView(1));
    else
        plotCoverageArea(plotters.SensorPlotter(i), sensors{i}.SensorLocation + shift,...
            sensors{i}.MaxRange, sensors{i}.Yaw,...
            sensors{i}.FieldOfView(1));
    end
end
end