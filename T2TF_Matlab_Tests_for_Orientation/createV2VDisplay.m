function [f,plotters] = createV2VDisplay(scenario, sensors, attachedVehicle)
% Create bird's eye plot
f = figure('Visible','off'); pause(0.1)
set(f,'Position',[433 350 1450 607]);
p1 = uipanel(f,'Title', 'Vehicle 1', 'Position', [0.01 0.01 0.39 0.98]);
p2 = uipanel(f,'Title', 'Vehicle 2', 'Position', [0.40 0.01 0.30 0.98]);
p3 = uipanel(f,'Title', 'Vehicle 3', 'Position', [0.70 0.01 0.30 0.98]);
a = axes(p1);
xlims = [0 100];
bep{1} = birdsEyePlot('Parent', a, 'XLim', xlims, 'YLim', [-50 50]);
a = axes(p2);
bep{2} = birdsEyePlot('Parent', a, 'XLim', xlims, 'YLim', [-50 50]);
a = axes(p3);
bep{3} = birdsEyePlot('Parent', a, 'XLim', xlims, 'YLim', [-50 50]);
set(p1.Children(1),'Position',[0.35 0.95 0.3 0.0345],'Orientation','horizontal','NumColumnsMode','manual','NumColumns',5);
set(p2.Children(1),'Position',[0.35 0.95 0.3 0.0345],'Orientation','horizontal','NumColumnsMode','manual','NumColumns',5);
set(p3.Children(1),'Position',[0.35 0.95 0.3 0.0345],'Orientation','horizontal','NumColumnsMode','manual','NumColumns',5);

for i = 1:numel(sensors)
    shift = scenario.Actors(attachedVehicle(i)).Position(1:2);
    name = "";
    if isa(sensors{i},'radarDataGenerator') || isa(sensors{i},'drivingRadarDataGenerator')
        plotters.SensorPlotter(i) = coverageAreaPlotter(bep{attachedVehicle(i)}, 'DisplayName',...
            name + " radar", 'FaceColor', [0.4660 0.6740 0.1880], 'FaceAlpha',0.1);
        plotCoverageArea(plotters.SensorPlotter(i), sensors{i}.MountingLocation(1:2) + shift,...
            sensors{i}.RangeLimits(2), sensors{i}.MountingAngles(3)+scenario.Actors(attachedVehicle(i)).Yaw,...
            sensors{i}.FieldOfView(1));
    else
        plotters.SensorPlotter(i) = coverageAreaPlotter(bep{attachedVehicle(i)}, 'DisplayName',...
            name + " vision", 'FaceColor', [0.3010 0.7450 0.9330]);
        plotCoverageArea(plotters.SensorPlotter(i), sensors{i}.SensorLocation + shift,...
            sensors{i}.MaxRange, sensors{i}.Yaw+scenario.Actors(attachedVehicle(i)).Yaw,...
            sensors{i}.FieldOfView(1));
    end
end
plotters.veh1DetPlotter = detectionPlotter(bep{1}, 'DisplayName', 'Detections',...
    'MarkerEdgeColor', '#FF00FF', 'Marker','o','MarkerSize',20);
plotters.veh2DetPlotter = detectionPlotter(bep{2}, 'DisplayName', 'Detections',...
    'MarkerEdgeColor', '#FF00FF', 'Marker','o','MarkerSize',20);
plotters.veh3DetPlotter = detectionPlotter(bep{3}, 'DisplayName', 'Detections',...
    'MarkerEdgeColor', '#FF00FF', 'Marker','o','MarkerSize',20);

% Local Tracks
plotters.veh1TrkPlotter = trackPlotter(bep{1}, 'DisplayName', 'Local Tracks',...
    'MarkerEdgeColor', 'red','Marker', '+','MarkerSize',30);
plotters.veh2TrkPlotter = trackPlotter(bep{2}, 'DisplayName', 'Local Tracks',...
    'MarkerEdgeColor', 'red','Marker', '+','MarkerSize',30);
plotters.veh3TrkPlotter = trackPlotter(bep{3}, 'DisplayName', 'Local Tracks',...
    'MarkerEdgeColor', 'red','Marker', '+','MarkerSize',30);

% Fuser Tracks
plotters.veh1FusePlotter = trackPlotter(bep{1}, 'DisplayName', 'Fuser Tracks','HistoryDepth',7,'Marker','square','MarkerSize',10);
plotters.ol1Plotter = outlinePlotter(bep{1});
plotters.lb1Plotter = laneBoundaryPlotter(bep{1});

plotters.veh2FusePlotter = trackPlotter(bep{2}, 'DisplayName', 'Fuser Tracks','HistoryDepth',7,'Marker','square','MarkerSize',10);
plotters.ol2Plotter = outlinePlotter(bep{2});
plotters.lb2Plotter = laneBoundaryPlotter(bep{2});

plotters.veh3FusePlotter = trackPlotter(bep{3}, 'DisplayName', 'Fuser Tracks','HistoryDepth',7,'Marker','square','MarkerSize',10);
plotters.ol3Plotter = outlinePlotter(bep{3});
plotters.lb3Plotter = laneBoundaryPlotter(bep{3});

rb = roadBoundaries(scenario);
[position, yaw, length, width, originOffset, color] = targetOutlines(scenario.Actors(1));
position = position + scenario.Actors(1).Position(1:2);

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
end