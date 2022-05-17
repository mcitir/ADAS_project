classdef HelperTrackDisplay < matlab.System
    % helperTrackDisplay Creates and updates the display
    %
    % This is a helper block for example purposes and may be removed or
    % modified in the future.
    %
    % Copyright 2019-2021 The MathWorks, Inc.
    
    %#codegen
    
    properties (Nontunable)
        % SubsystemBlockNames Name of the vehicles subsystems
        SubsystemBlockNames = {'Vehicle1', 'Vehicle2'};
        
        % PoseReaderBlockNames Name of the vehicles pose reader block
        PoseReaderBlockNames = {'Detection Concatenation', 'Vehicle2 Pose Reader'};
        
        % SourceTracksBlockNames Name of the source tracks data block
        SourceTracksBlockNames = {'JPDA Tracker V1', 'JPDA Tracker V2'};
        
        % SourceUpdatePoseBlockNames Name of the updated source block
        SourceUpdatePoseBlockNames = {'Update Tracks V1','Update Tracks V2'};
        
        % FusedTracksBlockNames Name of the fused tracks data block
        FusedTracksBlockNames = {'T2TF Tracker V1', 'T2TF Tracker V2'};
    end
    
    % RunTimeObjects for each block to access data during simulation
    properties (Access = protected)
        pPoseReaderBlock
        pSourceTracksBlock
        pFusedTracksBlock
        pSourceUpdatePoseBlock
    end
    
    properties
        %Pose Selector
        PosSelector = [1 0 0 0 0 0; 0 0 1 0 0 0;0 0 0 0 1 0];
    end
    
    properties(Access=private)
        pFig
        pBEP
        pPlotter
        pSimulinkUIToolbar
        pBlockName
        pIsLegendOn
        pScenario
        pSensors
        pAttachedVehicle
        pPlotters
        pV1
        pV2
        pVehicle1
        pVehicle2
        pHasObjectAttributes = false;
        pHasStateParameters = false;
    end
    
    methods
        function obj = HelperTrackDisplay(varargin)
            % Constructor
            setProperties(obj,nargin,varargin{:});
        end
        
        function set.PosSelector(obj,value)
            obj.PosSelector = value;
        end
        
        function value = get.PosSelector(obj)
            value = obj.PosSelector;
        end
        
    end
    
    methods(Access=protected)
        
        function setupImpl(obj,varargin)
            wasFigureClosed = (isempty(obj.pFig) || ~ishghandle(obj.pFig));
            obj.pBlockName = gcb;
            if wasFigureClosed
                % Find the hidden figure handle
                root = groot;
                shh = get(root,'ShowHiddenHandles');
                set(root,'ShowHiddenHandles','on');
                hfig = findobj('Tag',obj.pBlockName);
                set(root,'ShowHiddenHandles',shh);
                if isempty(hfig)
                    hfig = figure('Name',obj.pBlockName,'Tag',obj.pBlockName, ...
                        'CloseRequestFcn',@helperCloseReq,'Visible','off');
                    obj.pIsLegendOn = true;
                else % Hide the figure while we build the bird's-eye plot
                    set(hfig,'Visible','off')
                    obj.pIsLegendOn = ~isempty(get(hfig.CurrentAxes,'Legend'));
                end
                obj.pFig = hfig;
                modelName = bdroot;
                % Create scope toolbar
                if isempty(obj.pFig.UserData) % Toolbar got cleared
                    if isempty(obj.pSimulinkUIToolbar) % Toolbar was never created
                        t = findall(obj.pFig,'Type','uitoolbar');
                        if isempty(t)
                            t = uitoolbar(obj.pFig);
                        end
                        obj.pSimulinkUIToolbar = helperSimulinkUIToolbar(...
                            'Toolbar', t,...
                            'ModelName', modelName, ...
                            'BlockName', obj.pBlockName);
                    end
                    userData.SimulinkUIToolbar = obj.pSimulinkUIToolbar;
                    obj.pFig.UserData  = userData;
                else
                    obj.pSimulinkUIToolbar = obj.pFig.UserData.SimulinkUIToolbar;
                end
                % Turn off the legend if it was off earlier
                if ~obj.pIsLegendOn
                    legend(obj.pFig.CurrentAxes,'off')
                end
            end
            
            % Create BEP before the toolbar because clf clears toolbars
            isBEPNeeded = (isempty(obj.pBEP) || wasFigureClosed);
            if isBEPNeeded
                % Create the drivingScenario object and the two vehicles
                [scenario, vehicle1, vehicle2] = createDrivingScenario;
                obj.pScenario = scenario;
                obj.pVehicle1 = vehicle1;
                obj.pVehicle2 = vehicle2;
                % Create all the sensors
                [~, attachedVehicle] = createSensors(obj);
                obj.pAttachedVehicle = attachedVehicle;
                
                % Create display
                createT2TDisplay(obj);
            end
            % Define each vehicle as a vehicle, sensors, a tracker, and plotters
            obj.pV1 = struct('Actor', {obj.pVehicle1}, 'Sensors', {obj.pSensors(obj.pAttachedVehicle==1)}, 'DetPlotter', {obj.pPlotters.veh1DetPlotter}, 'TrkPlotter', {obj.pPlotters.veh1TrkPlotter});
            obj.pV2 = struct('Actor', {obj.pVehicle2}, 'Sensors', {obj.pSensors(obj.pAttachedVehicle==2)}, 'TrkPlotter', {obj.pPlotters.veh2TrkPlotter});
            
            % Setup the RunTimeObject for each block
            if isempty(obj.pPoseReaderBlock)
                for ii=1:numel(obj.PoseReaderBlockNames)
                    obj.pPoseReaderBlock{ii} = get_param([gcs,'/',obj.SubsystemBlockNames{ii},'/',obj.PoseReaderBlockNames{ii}],'RunTimeObject');
                end
            end
            if isempty(obj.pSourceTracksBlock)
                for j=1:numel(obj.SourceTracksBlockNames)
                    obj.pSourceTracksBlock{j} = get_param([gcs,'/',obj.SubsystemBlockNames{j},'/',obj.SourceTracksBlockNames{j}],'RunTimeObject');
                end
            end
            if isempty(obj.pFusedTracksBlock)
                for k=1:numel(obj.FusedTracksBlockNames)
                    obj.pFusedTracksBlock{k} = get_param([gcs,'/',obj.SubsystemBlockNames{k},'/',obj.FusedTracksBlockNames{k}],'RunTimeObject');
                end
            end
            if isempty(obj.pSourceUpdatePoseBlock)
                for l=1:numel(obj.SourceUpdatePoseBlockNames)
                    obj.pSourceUpdatePoseBlock{l} = get_param([gcs,'/',obj.SubsystemBlockNames{l},'/',obj.SourceUpdatePoseBlockNames{l}],'RunTimeObject');
                end
            end
            
        end
        
        function resetImpl(obj)
            % Bring the figure to front, set it to visible, but prevent
            % others from plotting to it
            isDirty = get_param(bdroot,'Dirty');
            set_param(obj.pBlockName,'UserData',obj.pFig)
            set_param(obj.pBlockName,'ModelCloseFcn','helperCloseAll(gcb)');
            set_param(bdroot,'Dirty',isDirty);
            set(obj.pFig,'Visible','on','HandleVisibility','off');
        end
        
        function releaseImpl(obj)
            % Release resources, such as file handles
            if ~isempty(obj.pFig.UserData)
                modelName = bdroot;
                isFastRestart = strcmp(get_param(modelName,'FastRestart'),'on');
                if isFastRestart %In fast restart mode, SimulationStatus is still 'running' at the end
                    setStoppedIcon(obj.pFig.UserData.SimulinkUIToolbar);
                else
                    update(obj.pFig.UserData.SimulinkUIToolbar);
                end
            end
            dirtyFlag = get_param(bdroot,'Dirty');
            set_param(obj.pBlockName,'OpenFcn','');
            set_param(bdroot,'Dirty',dirtyFlag);
        end
        
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            
            if isLocked(obj)
                s.pFig = obj.pFig;
                s.pBEP = obj.pBEP;
                s.pPlotter              = obj.pPlotter;
                s.pIsLegendOn           = obj.pIsLegendOn;
                s.pSimulinkUIToolbar    = saveobj(obj.pSimulinkUIToolbar);
            end
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            if wasLocked
                obj.pFig = s.pFig;
                obj.pBEP = s.pBEP;
                obj.pPlotter                = s.pPlotter;
                obj.pIsLegendOn             = s.pIsLegendOn;
                obj.pSimulinkUIToolbar      = loadobj(s.pSimulinkUIToolbar);
                s = rmfield(s,{'pFig','pBEP', ...
                    'pPlotter','pSimulinkUIToolbar'});
            end
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        
        function stepImpl(obj,varargin)
            
            [dets1,SourceTracks1,SourceTracks2,fusedTracks1,fusedTracks2] = getCurrentValues(obj);
            % Update the bird's-eye plot if it is visible
            if strcmp(get(obj.pFig,'Visible'),'on')
                
                time = get_param(bdroot,'SimulationTime'); %#ok<*NASGU>
                detsConsidered1 = dets1.Detections(1:dets1.NumDetections);
                % Plot detections
                if numel(detsConsidered1)>0
                    detPos = arrayfun(@(d)d.Measurement(1:2), detsConsidered1, 'UniformOutput', false);
                    detPos = cell2mat(detPos')' + obj.pV1.Actor.Position(1:2);
                    plotDetection(obj.pV1.DetPlotter, detPos);
                end
                
                tracksInScenario = SourceTracks1.Tracks(1:SourceTracks1.NumTracks);
                tracksInScenario = rmfield(tracksInScenario,{'StateSize','LogicSize'});
                for i = 1:SourceTracks1.NumTracks
                    obj.pHasObjectAttributes = isfield(tracksInScenario(i),'ObjectAttributes');
                    obj.pHasStateParameters = isfield(tracksInScenario(i),'StateParameters');
                    track = egoToScenario(tracksInScenario(i));
                    trs = optionalObjectAttributes(obj,track);
                    tr = optionalStateParameters(obj,trs);
                    tracksInScenario(i) = tr;
                end
                pos = getTrackPositions(tracksInScenario,obj.PosSelector);
                plotTrack(obj.pV1.TrkPlotter,pos);
                
                tracksInScenario2 = SourceTracks2.Tracks(1:SourceTracks2.NumTracks);
                tracksInScenario2 = rmfield(tracksInScenario2,{'StateSize','LogicSize'});
                for i = 1:SourceTracks2.NumTracks
                    obj.pHasObjectAttributes = isfield(tracksInScenario2(i),'ObjectAttributes');
                    obj.pHasStateParameters = isfield(tracksInScenario2(i),'StateParameters');
                    track = egoToScenario(tracksInScenario2(i));
                    trs = optionalObjectAttributes(obj,track);
                    tr = optionalStateParameters(obj,trs);
                    tracksInScenario2(i) = tr;
                end
                pos = getTrackPositions(tracksInScenario2,obj.PosSelector);
                plotTrack(obj.pV2.TrkPlotter,pos);
                if (fusedTracks1.NumTracks>0)
                    pos = getTrackPositions(fusedTracks1.Tracks(1:fusedTracks1.NumTracks),obj.PosSelector);
                    plotTrack(obj.pPlotters.veh1FusePlotter,pos);
                end
                if (fusedTracks2.NumTracks>0)
                    pos2 = getTrackPositions(fusedTracks2.Tracks(1:fusedTracks2.NumTracks),obj.PosSelector);
                    trs = fusedTracks2.Tracks(1:fusedTracks2.NumTracks);
                    ids = string([trs.TrackID]');
                    plotTrack(obj.pPlotters.veh2FusePlotter,pos2,ids);
                end
                % Update the display
                updateT2TDisplay(obj)
                
                % Advance the scenario one time step
                advance(obj.pScenario);
                
            end
        end
    end
    
    methods(Access=private)
        function createT2TDisplay(obj)
            %             scenario = obj.pScenario;
            % Create bird's eye plot
            set(obj.pFig,'Position',[433 425 1362 607]);
            p1 = uipanel(obj.pFig,'Title', 'Vehicle 1', 'Position', [0.01 0.01 0.48 0.98]);
            p2 = uipanel(obj.pFig,'Title', 'Vehicle 2', 'Position', [0.51 0.01 0.48 0.98]);
            a = axes(p1);
            xlims = [50 220];
            bep{1} = birdsEyePlot('Parent', a, 'XLim', xlims, 'YLim', [-50 50]);
            a = axes(p2);
            bep{2} = birdsEyePlot('Parent', a, 'XLim', xlims, 'YLim', [-50 50]);
            set(p1.Children(1),'Position',[0.35 0.95 0.3 0.0345],'Orientation','horizontal','NumColumnsMode','manual','NumColumns',5);
            set(p2.Children(1),'Position',[0.35 0.95 0.3 0.0345],'Orientation','horizontal','NumColumnsMode','manual','NumColumns',5);
            for i = 1:numel(obj.pSensors)
                shift = obj.pScenario.Actors(obj.pAttachedVehicle(i)).Position(1:2);
                name = "";
                if isa(obj.pSensors{i},'radarDataGenerator') || isa(obj.pSensors{i},'drivingRadarDataGenerator')
                    obj.pPlotters.SensorPlotter(i) = coverageAreaPlotter(bep{obj.pAttachedVehicle(i)}, 'DisplayName',...
                        name + " radar", 'FaceColor', 'r');
                    plotCoverageArea(obj.pPlotters.SensorPlotter(i), obj.pSensors{i}.MountingLocation(1:2) + shift,...
                        obj.pSensors{i}.RangeLimits(2), obj.pSensors{i}.MountingAngles(3),...
                        obj.pSensors{i}.FieldOfView(1));
                else
                    obj.pPlotters.SensorPlotter(i) = coverageAreaPlotter(bep{obj.pAttachedVehicle(i)}, 'DisplayName',...
                        name + " vision", 'FaceColor', 'b');
                    plotCoverageArea(obj.pPlotters.SensorPlotter(i), obj.pSensors{i}.SensorLocation + shift,...
                        obj.pSensors{i}.MaxRange, obj.pSensors{i}.Yaw,...
                        obj.pSensors{i}.FieldOfView(1));
                end
            end
            
            obj.pPlotters.veh1DetPlotter = detectionPlotter(bep{1}, 'DisplayName', 'Detections',...
                'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black');
            obj.pPlotters.veh1TrkPlotter = trackPlotter(bep{1}, 'DisplayName', 'Local Tracks',...
                'MarkerEdgeColor', 'black');
            obj.pPlotters.veh2TrkPlotter = trackPlotter(bep{2}, 'DisplayName', 'Local Tracks',...
                'MarkerEdgeColor', 'black','MarkerFaceColor', 'black');
            obj.pPlotters.veh1FusePlotter = trackPlotter(bep{1}, 'DisplayName', 'Fuser Tracks',...
                'MarkerEdgeColor', 'black', 'Marker', 'd');
            obj.pPlotters.veh2FusePlotter = trackPlotter(bep{2}, 'DisplayName', 'Fuser Tracks',...
                'MarkerEdgeColor', 'black', 'Marker', 'd');
            obj.pPlotters.ol1Plotter = outlinePlotter(bep{1});
            obj.pPlotters.lb1Plotter = laneBoundaryPlotter(bep{1});
            
            obj.pPlotters.ol2Plotter = outlinePlotter(bep{2});
            obj.pPlotters.lb2Plotter = laneBoundaryPlotter(bep{2});
            
            rb = roadBoundaries(obj.pScenario);
            [position, yaw, length, width, originOffset, color] = targetOutlines(obj.pScenario.Actors(1));
            position = position + obj.pScenario.Actors(1).Position(1:2);
            
            % update the bird's-eye plotters with the road and actors
            plotLaneBoundary(obj.pPlotters.lb1Plotter, rb);
            plotOutline(obj.pPlotters.ol1Plotter, position, yaw, length, width, ...
                'OriginOffset', originOffset, 'Color', color);
            plotLaneBoundary(obj.pPlotters.lb2Plotter, rb);
            plotOutline(obj.pPlotters.ol2Plotter, position, yaw, length, width, ...
                'OriginOffset', originOffset, 'Color', color);
        end
        
        function [numSensors, attachedVehicle] = createSensors(obj)
            % createSensors Returns all sensor objects to generate detections
            % Units used in createSensors and createDrivingScenario
            % Distance/Position - meters
            % Speed             - meters/second
            % Angles            - degrees
            % RCS Pattern       - dBsm
            
            % Assign into each sensor the physical and radar profiles for all actors
            profiles = actorProfiles(obj.pScenario);
            % Vehicle 1 radar reports clustered detections
            obj.pSensors{1} = radarDetectionGenerator('SensorIndex', 1, ...
                'SensorLocation', [3.7 0], 'MaxRange', 50, 'FieldOfView', [60 5], ...
                'ActorProfiles', profiles, 'HasOcclusion', true, 'HasFalseAlarms', false);
            
            % Vehicle 1 vision sensor reports detections
            obj.pSensors{2} = visionDetectionGenerator('SensorIndex', 2, ...
                'MaxRange', 100, 'SensorLocation', [1.9 0], 'DetectorOutput', 'Objects only', ...
                'ActorProfiles', profiles);

            % Vehicle 2 radar reports tracks
            obj.pSensors{3} = radarDetectionGenerator('SensorIndex', 3, ...
                'SensorLocation', [3.7 0], 'MaxRange', 120, 'FieldOfView', [30 5], ...
                'ActorProfiles', profiles, 'HasOcclusion', true, 'HasFalseAlarms', false);            
            
            attachedVehicle = [1;1;2];
            numSensors = numel(obj.pSensors);
        end
        
        function updateT2TDisplay(obj)
            % Updates the display of the road and actors on the road
            
            % get the road boundaries and rectangular outlines
            rb = roadBoundaries(obj.pScenario);
            [position, yaw, length, width, originOffset, color] = targetOutlines(obj.pScenario.Actors(1));
            position = position + obj.pScenario.Actors(1).Position(1:2);
            
            % update the bird's-eye plotters with the road and actors
            plotLaneBoundary(obj.pPlotters.lb1Plotter, rb);
            plotOutline(obj.pPlotters.ol1Plotter, position, yaw, length, width, ...
                'OriginOffset', originOffset, 'Color', color);
            plotLaneBoundary(obj.pPlotters.lb2Plotter, rb);
            plotOutline(obj.pPlotters.ol2Plotter, position, yaw, length, width, ...
                'OriginOffset', originOffset, 'Color', color);
            for i = 1:numel(obj.pSensors)
                shift = obj.pScenario.Actors(obj.pAttachedVehicle(i)).Position(1:2);
                if isa(obj.pSensors{i},'radarDataGenerator') || isa(obj.pSensors{i},'drivingRadarDataGenerator')
                    plotCoverageArea(obj.pPlotters.SensorPlotter(i), obj.pSensors{i}.MountingLocation(1:2) + shift,...
                        obj.pSensors{i}.RangeLimits(2), obj.pSensors{i}.MountingLocation(3),...
                        obj.pSensors{i}.FieldOfView(1));
                else
                    plotCoverageArea(obj.pPlotters.SensorPlotter(i), obj.pSensors{i}.SensorLocation + shift,...
                        obj.pSensors{i}.MaxRange, obj.pSensors{i}.Yaw,...
                        obj.pSensors{i}.FieldOfView(1));
                end
            end
            
        end
        
        function [dets1,trsV1,trsV2,fusedTrsV1,fusedTrsV2] = getCurrentValues(obj)
            % Get current values of inputs using RunTimeObjects of the
            % blocks.
            dets1 = obj.pPoseReaderBlock{1}.OutputPort(1).Data;
            TrsV1 = obj.pSourceTracksBlock{1}.OutputPort(1).Data;
            TrsV2 = obj.pSourceTracksBlock{2}.OutputPort(1).Data;
            fusedTrsV1 = obj.pFusedTracksBlock{1}.OutputPort(1).Data;
            fusedTrsV2 = obj.pFusedTracksBlock{2}.OutputPort(1).Data;
            UpdatedTrsV1 = obj.pSourceUpdatePoseBlock{1}.OutputPort(1).Data;
            UpdatedTrsV2 = obj.pSourceUpdatePoseBlock{2}.OutputPort(1).Data;
            trsV1 = getSourceTracks(obj,TrsV1,UpdatedTrsV1);
            trsV2 = getSourceTracks(obj,TrsV2,UpdatedTrsV2);
        end
        
        function trs = getSourceTracks(obj,x,y) %#ok<*INUSL>
            trs.NumTracks = x.NumTracks;
            trs.Tracks = y.Tracks;
        end
    end
    
    % Simulink interface
    methods(Access=protected)
        function str = getIconImpl(~)
            str = sprintf('Helper\n Track Display');
        end
        
        function num = getNumInputsImpl(~)
            % No inputs
            num = 0;
        end
        function varargout = getInputNamesImpl(obj)
            varargout = {};
            if obj.HasDetections1
                varargout = {varargout{:} sprintf('Vehicle1Detections')};
            end
            
            if obj.HasDetections2
                varargout = {varargout{:} sprintf('Vehicle2Detections')};
            end
            
            if obj.HasSourceTracksV1
                varargout = {varargout{:} sprintf('Vehicle1LocalTracks')};
            end
            if obj.HasSourceTracksV2
                varargout = {varargout{:} sprintf('Vehicle2LocalTracks')};
            end
            
            if obj.HasTracksFusedV1
                varargout = {varargout{:} sprintf('Vehicle1CentralTracks')};
            end
            
            if obj.HasTracksFusedV2
                varargout = {varargout{:} sprintf('Vehicle2CentralTracks')};
            end
        end
    end
    
    methods(Access=protected)
        function stOut = optionalObjectAttributes(obj,stIn)
            % Removes ObjectAttributes from output when
            % they are not provided in the input detections.
            
            if obj.pHasObjectAttributes
                stOut = stIn;
            else
                % Remove ObjectAttributes from struct
                if coder.target('MATLAB')
                    if ~obj.pHasObjectAttributes && isfield(stIn,'ObjectAttributes')
                        stOut = rmfield(stIn,'ObjectAttributes');
                    else
                        stOut = stIn;
                    end
                else
                    stOut = struct;
                    flds = fieldnames(stIn);
                    for m = 1:numel(flds)
                        thisFld = flds{m};
                        if ~obj.pHasObjectAttributes && strcmp(thisFld,'ObjectAttributes')
                            continue
                        end
                        stOut.(thisFld) = stIn.(thisFld);
                    end
                end
            end
        end
        
        function stOut = optionalStateParameters(obj,stIn)
            % Removes State Parameters field in simulink when it is an
            % empty struct.
            
            if  obj.pHasStateParameters
                stOut = stIn;
            else
                % Remove StateParameters from struct
                if coder.target('MATLAB')
                    if ~obj.pHasStateParameters && isfield(stIn,'StateParameters')
                        stOut = rmfield(stIn,'StateParameters');
                    else
                        stOut = stIn;
                    end
                else
                    stOut = struct;
                    flds = fieldnames(stIn);
                    for m = 1:numel(flds)
                        thisFld = flds{m};
                        if ~obj.pHasStateParameters && strcmp(thisFld,'StateParameters')
                            continue
                        end
                        stOut.(thisFld) = stIn.(thisFld);
                    end
                end
            end
        end
    end
    
    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title', 'Visualization',...
                'Text', getHeaderText());
        end
        
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = 'Interpreted execution';
        end
        
        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = false;
        end
    end
end

function str = getHeaderText
str = sprintf('Visualize Source and Central Tracks');
end



function [scenario, egoVehicle, secondVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario('SampleTime', 0.1);

% Add all road segments
roadCenters = [50.8 0.5 0; 253.4 1.5 0];
roadWidth = 12;
road(scenario, roadCenters, roadWidth);

roadCenters = [100.7 -100.6 0; 100.7 103.7 0];
road(scenario, roadCenters);

roadCenters = [201.1 -99.2 0; 199.7 99.5 0];
road(scenario, roadCenters);

% Add the ego vehicle
egoVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [65.1 -0.9 0]);
waypoints = [71 -0.5 0; 148.7 -0.5 0];
speed = 12;
trajectory(egoVehicle, waypoints, speed);

% Add the second vehicle
secondVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [55.1 -0.9 0]);
waypoints = [61 -0.5 0; 138.7 -0.5 0];
speed = 12;
trajectory(secondVehicle, waypoints, speed);

% Add the parked cars
vehicle(scenario, 'ClassID', 1, 'Position', [111.0 -3.6 0]);
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

function trackInScenario = egoToScenario(trackInEgo)
% Performs coordinate transformation from ego to scenario coordinates
% trackInEgo has StateParameters defined to transform it from ego
% coordinates to scenario coordinates
% We assume a constant velocity model with state [x;vx;y;vy;z;vz]
egoPosInScenario = trackInEgo.StateParameters.Position;
egoVelInScenario = trackInEgo.StateParameters.Velocity;
stateInScenario = trackInEgo.State;
stateShift = [egoPosInScenario(1);egoVelInScenario(1);egoPosInScenario(2);egoVelInScenario(2);egoPosInScenario(3);egoVelInScenario(3)];
stateInEgo = stateInScenario + stateShift;
trackInScenario = toStruct(objectTrack('UpdateTime',trackInEgo.UpdateTime,'State',stateInEgo,'StateCovariance',trackInEgo.StateCovariance,'StateParameters',trackInEgo.StateParameters));
if strcmpi(trackInScenario.TrackLogic,'History')
    trackInScenario.TrackLogic = trackLogicType(1);
elseif strcmpi(trackInScenario.TrackLogic,'Score')
    trackInScenario.TrackLogic = trackLogicType(2);
else
    trackInScenario.TrackLogic = trackLogicType(3);
end
end
