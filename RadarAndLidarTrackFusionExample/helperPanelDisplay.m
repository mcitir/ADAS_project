classdef helperPanelDisplay < matlab.System
    properties
        ColorOrder = standardColorOrder;
    end
    
    properties
        % Figure or panel to mount this display on
        Parent
    end
    
    properties
        GroundTruthColorIndex = 4;
        RadarTrackColorIndex = 1;
        LidarTrackColorIndex = 2;
        FusedTrackColorIndex = 9;
        RadarCoverageColorIndex = 2;
        LidarCoverageColorIndex = 3;
        GroundPointCloudColorIndex = 5;
        ObstaclePointCloudColorIndex = 4;
        CroppedPointCloudColorIndex = 3;
        EgoVehicleIndex = 1;
        EgoVehicleColorIndex = 7;
        LidarDetectionColorIndex = 7;
        RadarDetectionColorIndex = 6;
    end
    
    properties (Nontunable)
        PlotRadarTracks = true;
        PlotLidarTracks = true;
        PlotFusedTracks = true;
        PlotPointClouds = true;
        PlotSegmentedPointClouds = true;
        PlotRadarDetections = true;
        PlotLidarDetections = true;
        PlotCoverage = true;
        PlotRadarResolution = false;
        PlotGroundTruth = true;
        PlotRoads = true;
        PlotTrackVelocity = true;
        PlotRadarDetectionVelocity = false;
        PlotTrackCovariance = true;
        ShowLegend = true;
        FollowActorID
    end
    
    properties
        ViewLocation = [-100 0];
        ViewHeight = 100;
        ViewYaw = 0;
        ViewPitch = 60;
        ViewRoll = 0;
    end
    
    properties (Dependent, SetAccess = protected)
        GroundTruthColor
        RadarTrackColor
        LidarTrackColor
        FusedTrackColor
        RadarCoverageColor
        LidarCoverageColor
        GroundPointCloudColor
        ObstaclePointCloudColor
        CroppedPointCloudColor
        EgoVehicleColor
        LidarDetectionColor
        RadarDetectionColor
    end
    
    properties
        pGroundTruthPlotter
        pRadarTrackPlotter
        pRadarDetectionPlotter
        pLidarTrackPlotter
        pLidarDetectionPlotter
        pFusedTrackPlotter
        pCoveragePlotter
        pGroundPointCloudPlotter
        pObstaclePointCloudPlotter
        pCroppedPointCloudPlotter
        pTheaterPlot
        pBirdsEyePlot
        pFigure
        pDimStruct = struct('Length',0,'Width',0,'Height',0,'OriginOffset',[0 0 0]);
        pCoverageConfigStruct = coverageConfig(radarSensor(1));
        pScenarioPlot
    end
    properties
        pResolutionPlotters
    end
    
    methods
        function val = get.GroundTruthColor(obj)
            val = obj.ColorOrder(obj.GroundTruthColorIndex,:);
        end
        function val = get.RadarTrackColor(obj)
            val = obj.ColorOrder(obj.RadarTrackColorIndex,:);
        end
        function val = get.LidarTrackColor(obj)
            val = obj.ColorOrder(obj.LidarTrackColorIndex,:);
        end
        function val = get.FusedTrackColor(obj)
            val = obj.ColorOrder(obj.FusedTrackColorIndex,:);
        end
        function val = get.RadarCoverageColor(obj)
            val = obj.ColorOrder(obj.RadarCoverageColorIndex,:);
        end
        function val = get.LidarCoverageColor(obj)
            val = obj.ColorOrder(obj.LidarCoverageColorIndex,:);
        end
        function val = get.GroundPointCloudColor(obj)
            val = obj.ColorOrder(obj.GroundPointCloudColorIndex,:);
        end
        function val = get.EgoVehicleColor(obj)
            val = obj.ColorOrder(obj.EgoVehicleColorIndex,:);
        end
        function val = get.LidarDetectionColor(obj)
            val = obj.ColorOrder(obj.LidarDetectionColorIndex,:);
        end
        function val = get.RadarDetectionColor(obj)
            val = obj.ColorOrder(obj.RadarDetectionColorIndex,:);
        end
        function val = get.ObstaclePointCloudColor(obj)
            val = obj.ColorOrder(obj.ObstaclePointCloudColorIndex,:);
        end
        function val = get.CroppedPointCloudColor(obj)
            val = obj.ColorOrder(obj.CroppedPointCloudColorIndex,:);
        end
    end
    
    methods
        function obj = helperPanelDisplay(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access = {?helperLidarRadarTrackFusionDisplay})
        function showLegend(obj)
            % Show legend of existing figure on a new axes
            assert(obj.ShowLegend,'Legend must exist');
            ax = obj.pTheaterPlot.Parent;
            ax.Position = [0 0 0 0];
            l = legend(ax);
            l.Position = [0 0 1 1];
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj, varargin)
            setupPlotters(obj, varargin{:});
        end
        
        function stepImpl(obj, scenario, ...
                radarSensors, radarDetections, radarTracks, ...
                lidarSensors, ptClouds, lidarDetections, segmentationInfo, lidarTracks, ...
                fusedTracks)

            % Plot truth
            plotTruth(obj, scenario, radarSensors, lidarSensors);
            
            % Plot detection data
            plotSensorData(obj, scenario, radarDetections, ptClouds, lidarDetections, segmentationInfo);
            
            % Plot track data
            plotTracks(obj, radarTracks, lidarTracks, fusedTracks);
        end
    end
    
    %% Plot updates
    methods
        function plotTruth(obj, scenario, radarSensors, lidarSensors)
            if obj.PlotGroundTruth
                obj.pScenarioPlot.plotActors(scenario.Actors);
            else
                obj.pScenarioPlot.plotActors(driving.scenario.Actor.empty(0,1));
            end
            
            if obj.PlotCoverage
                [pos, range, orientation, fov] = parseCoverageAreaConfigs(obj, scenario, radarSensors, lidarSensors);
                for i = 1:numel(obj.pCoveragePlotter)
                    obj.pCoveragePlotter{i}.plotCoverageArea(pos(i,:),range(i),orientation(i),fov(i));
                end
                % Plot radar resolutions
                if obj.PlotRadarResolution
                    plotResolutions(obj, scenario, radarSensors);
                end
            end
        end
        
        function plotResolutions(obj, scenario, radarSensors)
           ego = scenario.Actors(obj.EgoVehicleIndex);
           egoPos = ego.Position;
           egoOrient = quaternion([ego.Yaw ego.Pitch ego.Roll],'eulerd','ZYX','frame');
           numPlotters = numel(radarSensors);
           if isempty(obj.pResolutionPlotters)
              obj.pResolutionPlotters = cell(numPlotters,1);
              for i = 1:numPlotters
                  obj.pResolutionPlotters{i} = plot3(nan,nan,nan,'Color',obj.RadarCoverageColor);
              end
           end
           for i = 1:numel(radarSensors)
               sensorPos = [radarSensors{i}.SensorLocation radarSensors{i}.Height];
               sensorOrient = quaternion([radarSensors{i}.Yaw radarSensors{i}.Pitch radarSensors{i}.Roll],'eulerd','ZYX','frame');
               fov = radarSensors{1}.FieldOfView(1);
               azRes = radarSensors{i}.AzimuthResolution;
               maxRange = radarSensors{i}.MaxRange;
               rRes = radarSensors{i}.RangeResolution;
               az = deg2rad((-fov(1)/2:azRes:fov(1)/2));
               
               r = (0:rRes:maxRange);
               
               resLines = zeros(0,2);
               
               for m = 1:numel(az)
                   resLines = [resLines;0 az(m);maxRange az(m);nan az(m)]; %#ok<AGROW>
               end
               for m = 1:numel(r)
                   for k = 1:numel(az)
                       resLines = [resLines;r(m) az(k)]; %#ok<AGROW>
                   end
                   resLines = [resLines;nan nan]; %#ok<AGROW>
               end
               
               [resLinesX,resLinesY,resLinesZ] = sph2cart(resLines(:,2),0*resLines(:,2),resLines(:,1));
               resLinesXYZ = [resLinesX resLinesY resLinesZ];
               resLinesXYZ = rotatepoint(sensorOrient,resLinesXYZ) + sensorPos;
               resLinesXYZ = rotatepoint(egoOrient,resLinesXYZ) + egoPos;
               
               obj.pResolutionPlotters{i}.XData = resLinesXYZ(:,1);
               obj.pResolutionPlotters{i}.YData = resLinesXYZ(:,2);
               obj.pResolutionPlotters{i}.ZData = resLinesXYZ(:,3);
           end
        end
        
        function plotSensorData(obj, scenario, radarDetections, ptCloud, lidarDetections, segmentationInfo)
            ego = scenario.Actors(obj.EgoVehicleIndex);
            egoPos = ego.Position;
            egoOrient = quaternion([ego.Yaw ego.Pitch ego.Roll],'eulerd','ZYX','frame');
            
            loc = rotatepoint(egoOrient,ptCloud.Location) + egoPos;
            ptCloud = pointCloud(loc);
                
            if obj.PlotPointClouds && obj.PlotSegmentedPointClouds
                groundCloud = select(ptCloud, segmentationInfo.GroundIndices);
                obstacleCloud = select(ptCloud, segmentationInfo.ObstacleIndices);
                croppedCloud = select(ptCloud, segmentationInfo.CroppedIndices);
                
                plotPointCloud(obj.pGroundPointCloudPlotter,groundCloud);
                plotPointCloud(obj.pObstaclePointCloudPlotter,obstacleCloud);
                plotPointCloud(obj.pCroppedPointCloudPlotter,croppedCloud);
            elseif obj.PlotPointClouds
                groundCloud = select(ptCloud, segmentationInfo.GroundIndices);
                obstacleCloud = select(ptCloud, segmentationInfo.ObstacleIndices);
                plotPointCloud(obj.pGroundPointCloudPlotter, groundCloud);
                plotPointCloud(obj.pObstaclePointCloudPlotter,obstacleCloud);
            end
            
            if obj.PlotRadarDetections
                [pos, vel] = parseRadarDetections(obj, ego, radarDetections);
                plotDetection(obj.pRadarDetectionPlotter,pos,vel);
            end
            
            if obj.PlotLidarDetections
                [pos, dim, orient] = parseLidarDetections(obj, ego, lidarDetections);
                plotTrack(obj.pLidarDetectionPlotter,pos,dim,orient);
            end
        end
        
        function plotTracks(obj, radarTracks, lidarTracks, fusedTracks)
            if obj.PlotLidarTracks
                [pos, vel, dim, orient, trkIDs, labels, posCov] = parseLidarTracks(obj, lidarTracks);
                if ~obj.PlotTrackVelocity
                    vel(:) = nan;
                end
                if ~obj.PlotTrackCovariance
                    plotTrack(obj.pLidarTrackPlotter,pos,vel,dim,orient,trkIDs,labels);
                else
                    plotTrack(obj.pLidarTrackPlotter,pos,posCov,vel,dim,orient,trkIDs,labels);
                end
            end
            
            if obj.PlotRadarTracks
                [pos, vel, dim, orient, trkIDs, labels, posCov] = parseRadarTracks(obj, radarTracks);
                if ~obj.PlotTrackVelocity
                    vel(:) = nan;
                end
                if ~obj.PlotTrackCovariance
                    plotTrack(obj.pRadarTrackPlotter,pos,vel,dim,orient,trkIDs,labels);
                else
                    plotTrack(obj.pRadarTrackPlotter,pos,posCov,vel,dim,orient,trkIDs,labels);
                end
            end
            
            if obj.PlotFusedTracks
                [pos, vel, dim, orient, trkIDs, labels, posCov] = parseFusedTracks(obj, fusedTracks);
                if ~obj.PlotTrackVelocity
                    vel(:) = nan;
                end
                if ~obj.PlotTrackCovariance
                    plotTrack(obj.pFusedTrackPlotter,pos,vel,dim,orient,trkIDs,labels);
                else
                    plotTrack(obj.pFusedTrackPlotter,pos,posCov,vel,dim,orient,trkIDs,labels);
                end
            end
        end
    end
    
    %% Plot setups
    methods (Hidden)
        function setupPlotters(obj, scenario, ...
                radarSensors, ~, ~, ...
                lidarSensors, varargin)
            ax = createAxes(obj);
            
            createGroundTruthPlotter(obj, ax, scenario, radarSensors, lidarSensors);
            
            createTrackPlotters(obj);
            
            createPointCloudPlotters(obj);
            
            createDetectionPlotters(obj);
            l = legend(ax);
            if ~obj.ShowLegend
                delete(l);
            end
        end
        
        function ax = createAxes(obj)
            if isempty(obj.Parent)
                obj.Parent = figure('Units','normalized','Position',[0.1 0.1 0.8 0.8]);
            end
            
            ax = axes(obj.Parent);
            ax.Camera.TransparencyMethodHint = "objectsort";
            
            if obj.ColorOrder(1,1) == 1  %Dark-mode
                if isa(obj.Parent,'matlab.ui.Figure')
                    obj.Parent.Color = 0.157*[1 1 1];
                elseif isa(obj.Parent,'matlab.ui.container.Panel')
                    obj.Parent.BackgroundColor = 0.157*[1 1 1];
                    obj.Parent.ForegroundColor = [1 1 1];
                end
                ax.Color = [0 0 0];
                grid(ax,'on');
                ax.GridColor = 0.68*[1 1 1];
                ax.GridAlpha = 0.4;
                axis(ax,'equal');
                ax.XColor = [1 1 1]*0.68;
                ax.YColor = [1 1 1]*0.68;
                ax.ZColor = [1 1 1]*0.68;
            else
                if isa(obj.Parent,'matlab.ui.container.Panel')
                    obj.Parent.BackgroundColor = [1 1 1];
                end
            end
            obj.pTheaterPlot = theaterPlot('Parent',ax);
            obj.pBirdsEyePlot = birdsEyePlot('Parent',ax);
            
            l = legend(ax);
            l.TextColor = 1 - ax.Color;
        end
        
        function createPointCloudPlotters(obj)
            if obj.PlotPointClouds && obj.PlotSegmentedPointClouds
                obj.pGroundPointCloudPlotter = pointCloudPlotter(obj.pBirdsEyePlot,'Color',obj.GroundPointCloudColor,'PointSize',0.5,'DisplayName','Ground Point Cloud');
                obj.pObstaclePointCloudPlotter = pointCloudPlotter(obj.pBirdsEyePlot,'Color',obj.ObstaclePointCloudColor,'PointSize',2,'DisplayName','Obstacle Point Cloud');
                obj.pCroppedPointCloudPlotter = pointCloudPlotter(obj.pBirdsEyePlot,'Color',obj.CroppedPointCloudColor,'PointSize',0.1,'DisplayName','Cropped Point Cloud');
            elseif obj.PlotPointClouds
                obj.pGroundPointCloudPlotter = pointCloudPlotter(obj.pBirdsEyePlot,'Color',obj.GroundPointCloudColor,'PointSize',0.5);
                pointCloudPlotter(obj.pBirdsEyePlot,'Color',obj.GroundPointCloudColor,'PointSize',2,'DisplayName','Point Cloud');
                obj.pObstaclePointCloudPlotter = pointCloudPlotter(obj.pBirdsEyePlot,'Color',obj.GroundPointCloudColor,'PointSize',1);
            end
        end
        
        function createGroundTruthPlotter(obj, ax, scenario, radarSensors, lidarSensors)
            
            if isempty(obj.FollowActorID)
                panOn = scenario.Actors(obj.EgoVehicleIndex);
            else
                panOn = scenario.Actors(obj.FollowActorID);
            end
            
            p = driving.scenario.Plot('Parent',ax,'EgoActor',panOn,...
                'ViewHeight', obj.ViewHeight, ...
                'ViewLocation', obj.ViewLocation, ...
                'ViewRoll', obj.ViewRoll, ...
                'ViewPitch', obj.ViewPitch, ...
                'ViewYaw', obj.ViewYaw,...
                'Meshes','on');
            
            obj.pScenarioPlot = p;
            
            if obj.PlotGroundTruth
                l = legend(ax);
                l.AutoUpdate = 'off';
                
                if obj.PlotRoads
                    p.plotRoads(scenario.RoadTiles, roadBoundaries(scenario),scenario.RoadCenters,scenario.ShowRoadBorders);
                    roadPatches = findall(ax,'Tag','RoadTilesPatch');
                    for i = 1:numel(roadPatches)
                        if obj.ColorOrder(1) == 1
                            roadPatches(i).FaceColor = [0.1 0.1 0.1];
                            roadPatches(i).EdgeColor = [0 0 0];
                        end
                    end
                end
                scenario.Actors(obj.EgoVehicleIndex).PlotColor = obj.EgoVehicleColor;
                
                p.plotActors(scenario.Actors);
                allPatches = findall(ax,'Type','Patch');
                allTags = arrayfun(@(x)x.Tag,allPatches,'UniformOutput',false);
                actorPatches = allPatches(startsWith(allTags,'ActorPatch'));
                
                l.AutoUpdate = 'on';
                for i = 1:numel(scenario.Actors)
                    scenario.Actors(i).PlotColor = obj.GroundTruthColor;
                end
                
                for i = 1:numel(actorPatches)
                    actorPatches(i).FaceColor = obj.GroundTruthColor;
                    actorPatches(i).FaceAlpha = 0.3;
                    actorPatches(i).EdgeAlpha = 0.1;
                end
                
                hold (ax,'on');
                patch(ax,nan,nan,nan,'FaceColor',obj.GroundTruthColor,'DisplayName','Ground Truth');
                
            else
                obj.pScenarioPlot.plotActors(driving.scenario.Actor.empty(0,1));
            end
            
            createCoveragePlotters(obj, scenario, radarSensors, lidarSensors);
        end
        
        function createDetectionPlotters(obj)
            if obj.PlotRadarDetections
                obj.pRadarDetectionPlotter = detectionPlotter(obj.pTheaterPlot,'MarkerFaceColor',obj.RadarDetectionColor,...
                    'DisplayName','Radar Detections');
            end
            if obj.PlotLidarDetections
                obj.pLidarDetectionPlotter = trackPlotter(obj.pTheaterPlot,...
                    'MarkerFaceColor',obj.LidarDetectionColor,...
                    'MarkerEdgeColor',obj.LidarDetectionColor,...
                    'DisplayName','Lidar Bounding Box Detections',...
                    'Marker','^',...
                    'MarkerSize',4);
            end
        end
        
        function createCoveragePlotters(obj, scenario, radarSensors, lidarSensors)
%             if obj.PlotCoverage
%                 obj.pCoveragePlotter = coveragePlotter(obj.pTheaterPlot,'Alpha',[0 0],'DisplayName','Coverage');
%                 [covConfig, indices, colors] = parseCoverageConfigs(obj, scenario, radarSensors, {});
%                 plotCoverage(obj.pCoveragePlotter, covConfig, indices, colors);
%                 currentPatches = findall(obj.pTheaterPlot.Parent,'Type','Patch');
%                 allTags = arrayfun(@(x)x.Tag,currentPatches,'UniformOutput',false);
%                 beamPatches = currentPatches(startsWith(allTags,'coverage'));
%                 for i = 1:numel(beamPatches)
%                     beamPatches(i).FaceAlpha = 0;
%                     beamPatches(i).EdgeAlpha = 1;
%                     beamPatches(i).EdgeColor = beamPatches(i).FaceColor;
%                 end
%             end
            if obj.PlotCoverage
                [pos, ~, ~, ~, colors] = parseCoverageAreaConfigs(obj, scenario, radarSensors, lidarSensors);
                obj.pCoveragePlotter = cell(size(pos,1),1);
                for i = 1:numel(obj.pCoveragePlotter)
                    obj.pCoveragePlotter{i} = coverageAreaPlotter(obj.pBirdsEyePlot,'EdgeColor',colors{i},...
                        'FaceColor',colors{i});
                end
            end
        end
        
        function createTrackPlotters(obj, varargin)
            pos = nan(1,3);
            vel = nan(1,3);
            dim = repmat(obj.pDimStruct,1,1);
            orient = repmat(quaternion([1 0 0 0]),1,1);
            trkID = 1e6;
            labels = "F_1";
            
            if obj.PlotRadarTracks
                obj.pRadarTrackPlotter = trackPlotter(obj.pTheaterPlot,...
                    'MarkerFaceColor',obj.RadarTrackColor,...
                    'MarkerEdgeColor',obj.RadarTrackColor,...
                    'ConnectHistory','off',...
                    'DisplayName','Radar Tracks',...
                    'LabelOffset',[1 0 0],...
                    'FontSize',15);
                plotTrack(obj.pRadarTrackPlotter,pos,vel,dim,orient,trkID,labels);
            end
            if obj.PlotLidarTracks
                obj.pLidarTrackPlotter = trackPlotter(obj.pTheaterPlot,...
                    'MarkerFaceColor',obj.LidarTrackColor,...
                    'MarkerEdgeColor',obj.LidarTrackColor,...
                    'ConnectHistory','off',...
                    'DisplayName','Lidar Tracks',...
                    'LabelOffset',[1 0 0],...
                    'FontSize',15);
                plotTrack(obj.pLidarTrackPlotter,pos,vel,dim,orient,trkID,labels);
            end
            if obj.PlotFusedTracks
                obj.pFusedTrackPlotter = trackPlotter(obj.pTheaterPlot,...
                    'MarkerFaceColor',obj.FusedTrackColor,...
                    'MarkerEdgeColor',obj.FusedTrackColor,...
                    'ConnectHistory','off',...
                    'DisplayName','Fused Tracks',...
                    'LabelOffset',[0 2 2],...
                    'FontSize',15);
                plotTrack(obj.pFusedTrackPlotter,pos,vel,dim,orient,trkID,labels);
            end
            
            % Customize the patches by getting their handles through tags
            trackPatches = findall(obj.pTheaterPlot.Parent,'Tag','tpTrackExtent');
            
            for i = 1:numel(trackPatches)
                trackPatches(i).FaceAlpha = 0;
                trackPatches(i).EdgeAlpha = 1;
                trackPatches(i).LineWidth = 2;
            end
        end
    end
    
    %% Parsing methods
    methods (Access = protected)
        function [pos, range, orient, fov, colors] = parseCoverageAreaConfigs(obj, scenario, radarSensors, lidarSensors)
           [covConfig, ~, colors] = parseCoverageConfigs(obj, scenario, radarSensors, lidarSensors); 
           n = numel(covConfig);
           pos = zeros(n,2);
           range = zeros(n,1);
           fov = zeros(n,1);
           orient = zeros(n,1);
           for i = 1:n
               pos(i,:) = covConfig(i).Position(1:2);
               range(i) = covConfig(i).Range;
               fov(i) = covConfig(i).FieldOfView(1);
               ypr = eulerd(covConfig(i).Orientation,'ZYX','frame');
               orient(i) = ypr(1);
           end
        end
        
        function [covConfig, indices, colors] = parseCoverageConfigs(obj, scenario, radarSensors, lidarSensors)
            numRadars = numel(radarSensors);
            numLidars = numel(lidarSensors);
            covConfig = repmat(obj.pCoverageConfigStruct,numRadars + numLidars,1);
            indices = zeros(numRadars + numLidars, 1);
            colors = cell(numRadars + numLidars, 1);
            ego = scenario.Actors(obj.EgoVehicleIndex);
            egoPos = scenario.Actors(obj.EgoVehicleIndex).Position;
            egoOrient = quaternion([ego.Yaw ego.Pitch ego.Roll],'eulerd','ZYX','frame');
            
            for i = 1:numRadars
                indices(i) = radarSensors{i}.SensorIndex;
                covConfig(i).Index = radarSensors{i}.SensorIndex;
                covConfig(i).LookAngle = [0 0];
                covConfig(i).FieldOfView = radarSensors{i}.FieldOfView;
                covConfig(i).ScanLimits = [-covConfig(i).FieldOfView(1)/2 covConfig(i).FieldOfView(1)/2;...
                    -covConfig(i).FieldOfView(2)/2 covConfig(i).FieldOfView(2)/2];
                covConfig(i).Range = radarSensors{i}.RangeLimits(2);
                mountingPos = radarSensors{i}.MountingLocation(:);
                mountingOrient = quaternion(radarSensors{i}.MountingAngles,'eulerd','ZYX','frame');
                pos = egoPos + rotatepoint(egoOrient,mountingPos');
                orient = egoOrient*mountingOrient;
                covConfig(i).Position = pos;
                covConfig(i).Orientation = orient;
                colors{i} = obj.RadarCoverageColor;
            end
            
            if isscalar(lidarSensors)
                lidarSensors = {lidarSensors};
            end
            
            for i = 1:numLidars
                k = numRadars + i;
                indices(k) = lidarSensors{i}.SensorIndex;
                covConfig(k).Index = lidarSensors{i}.SensorIndex;
                covConfig(k).LookAngle = [mean(lidarSensors{i}.AzimuthLimits);mean(lidarSensors{i}.ElevationLimits)];
                covConfig(k).FieldOfView = [diff(lidarSensors{i}.AzimuthLimits);diff(lidarSensors{i}.ElevationLimits)];
                covConfig(k).ScanLimits = [lidarSensors{i}.AzimuthLimits + [-10 10];lidarSensors{i}.ElevationLimits];
                covConfig(k).Range = lidarSensors{i}.MaxRange;
                mountingPos = [lidarSensors{i}.SensorLocation(:);lidarSensors{i}.Height];
                mountingOrient = quaternion([lidarSensors{i}.Yaw lidarSensors{i}.Pitch lidarSensors{i}.Roll],'eulerd','ZYX','frame');
                pos = egoPos + rotatepoint(egoOrient,mountingPos');
                orient = egoOrient*mountingOrient;
                covConfig(k).Position = pos;
                covConfig(k).Orientation = orient;
                colors{k} = obj.LidarCoverageColor;
            end
        end
        
        function [pos, vel] = parseRadarDetections(obj, ego, radarDetections)
            n = numel(radarDetections);
            posEgo = zeros(n,3);
            velEgo = zeros(n,3);
            egoOrient = quaternion([ego.Yaw ego.Pitch ego.Roll],'eulerd','ZYX','frame');
            egoPos = ego.Position;
            egoVel = ego.Velocity;
            for i = 1:n
                [posEgo(i,:),velEgo(i,:)] = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(radarDetections{i},'rtFusionDisplay','double');
            end
            pos = egoPos + rotatepoint(egoOrient,posEgo);
            vel = egoVel + rotatepoint(egoOrient,velEgo);
            if ~obj.PlotRadarDetectionVelocity
                vel(:) = 0;
            end
        end
        
        function [pos, dim, orient] = parseLidarDetections(obj, ego, lidarDetections)
            n = numel(lidarDetections);
            
            pos = zeros(n,3);
            dim = repmat(obj.pDimStruct,n,1);
            egoOrient = quaternion([ego.Yaw ego.Pitch ego.Roll],'eulerd','ZYX','frame');
            egoPos = ego.Position;
            orient = repmat(egoOrient,n,1);
            
            for i = 1:n
                pos(i,:) = lidarDetections{i}.Measurement(1:3);
                dim(i).Length = lidarDetections{i}.Measurement(5);
                dim(i).Width = lidarDetections{i}.Measurement(6);
                dim(i).Height = lidarDetections{i}.Measurement(7);
                detOrient = quaternion([lidarDetections{i}.Measurement(4) 0 0],'eulerd','ZYX','frame');
                orient(i) = detOrient*orient(i);
            end
            pos = egoPos + rotatepoint(egoOrient, pos);
        end
        
        function [pos, vel, dim, orient] = parseTruth(obj, scenario)
            numTruth = numel(scenario.Actors);
            pos = zeros(3,numTruth)';
            vel = zeros(3,numTruth)';
            dim = repmat(obj.pDimStruct,numTruth,1);
            orient = repmat(quaternion([1 0 0 0]),numTruth,1);
            for i = 1:numTruth
                tr = scenario.Actors(i);
                pos(i,:) = tr.Position;
                vel(i,:) = tr.Velocity;
                dim(i) = dimensions(tr);
                orient(i) = quaternion([tr.Yaw tr.Pitch tr.Roll],'eulerd','ZYX','frame');
            end
        end
        
        function [pos, vel, dim, orient, trkIDs, labels, posCov] = parseRadarTracks(obj,radarTracks)
            % Radar tracks are from GM-PHD with ctrect convention
            numTracks = numel(radarTracks);
            pos = zeros(3,numTracks)';
            vel = zeros(3,numTracks)';
            dim = repmat(obj.pDimStruct,numTracks,1);
            orient = repmat(quaternion([1 0 0 0]),numTracks,1);
            trkIDs = zeros(1,numTracks);
            labels = repmat(string,1,numTracks);
            posCov = zeros(3,3,numTracks);
            for i = 1:numTracks
                trk = radarTracks(i);
                pos(i,:) = [trk.State(1);trk.State(2);0];
                posCov(:,:,i) = blkdiag(trk.StateCovariance(1:2,1:2),1); % Unit covariance in z for display
                vel(i,:) = [trk.State(3)*cosd(trk.State(4));trk.State(3)*sind(trk.State(4));0];
                dim(i).Length = trk.State(6);
                dim(i).Width = trk.State(7);
                dim(i).Height = 0;
                orient(i) = quaternion([trk.State(4) 0 0],'eulerd','ZYX','frame');
                trkIDs(i) = trk.TrackID;
                labels(i) = strcat("R",num2str(trk.TrackID));
            end
        end
        
        function [pos, vel, dim, orient, trkIDs, labels, posCov] = parseLidarTracks(obj,lidarTracks)
            % Lidar tracks are from JPDA with constturn cuboid convention
            numTracks = numel(lidarTracks);
            pos = zeros(3,numTracks)';
            vel = zeros(3,numTracks)';
            dim = repmat(obj.pDimStruct,numTracks,1);
            orient = repmat(quaternion([1 0 0 0]),numTracks,1);
            trkIDs = zeros(1,numTracks);
            labels = repmat(string,1,numTracks);
            posCov = zeros(3,3,numTracks);
            for i = 1:numTracks
                trk = lidarTracks(i);
                pos(i,:) = [trk.State(1);trk.State(2);trk.State(6)];
                posCov(:,:,i) = trk.StateCovariance([1 2 6],[1 2 6]);
                vel(i,:) = [trk.State(3)*cosd(trk.State(4));trk.State(3)*sind(trk.State(4));trk.State(7)];
                dim(i).Length = trk.State(8);
                dim(i).Width = trk.State(9);
                dim(i).Height = trk.State(10);
                orient(i) = quaternion([trk.State(4) 0 0],'eulerd','ZYX','frame');
                trkIDs(i) = trk.TrackID;
                labels(i) = strcat("L",num2str(trk.TrackID));
            end
        end
        
        function [pos, vel, dim, orient, trkIDs, labels, posCov] = parseFusedTracks(obj, fusedTracks)
            [pos, vel, dim, orient, trkIDs, labels, posCov] = parseLidarTracks(obj, fusedTracks);
            labels = strrep(labels,"L","F");
        end
        
    end
end

function colorOrder = darkColorOrder
colorOrder = [1.0000    1.0000    0.0667
    0.0745    0.6235    1.0000
    1.0000    0.4118    0.1608
    0.3922    0.8314    0.0745
    0.7176    0.2745    1.0000
    0.0588    1.0000    1.0000
    1.0000    0.0745    0.6510];

colorOrder(8,:) = [1 1 1];
colorOrder(9,:) = [0 0 0];
colorOrder(10,:) = 0.7*[1 1 1];
end

function colorOrder = standardColorOrder
colorOrder = lines(7);
colorOrder(8,:) = [1 1 1];
colorOrder(9,:) = [0 0 0];
colorOrder(10,:) = 0.3*[1 1 1];
end
