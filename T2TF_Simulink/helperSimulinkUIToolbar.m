classdef helperSimulinkUIToolbar < handle
    %helperSimulinkUIToolbar Creates pushtools to control a Simulink model
    
    % Copyright 2019 The MathWorks Inc.
    
    properties
        Toolbar
        ModelName
        BlockName
    end
    
    properties (Access=protected)
        StartIcon
        PauseIcon
        StepForwardIcon
        StepForwardInactiveIcon
        StopIcon
        StopInactiveIcon
        StartPausePushtool
        StepForwardPushtool
        StopPushtool
        StartPauseClicked = false
    end
    
    methods
        function obj = helperSimulinkUIToolbar(varargin)
            % Assign name/value pairs on construction
            for i=1:2:numel(varargin)
                obj.(varargin{i}) = varargin{i+1};
            end
            
            modifyRegularToolbar(obj);
            reset(obj);
            update(obj);
        end
        
        function modifyRegularToolbar(obj)
            % Disable figure menu bar
            hfig = obj.Toolbar.Parent;
            addToolbarExplorationButtons(hfig);
            set(hfig,'DefaultAxesToolbarVisible','off');
            set(hfig,'MenuBar','none','Toolbar','figure','DockControl','off');
            
            % Disable figure buttons that are not needed
            tagsToKeep = {'Annotation.InsertLegend',...
                'Exploration.Pan',...
                'Exploration.ZoomOut',...
                'Exploration.ZoomIn',...
                'SimulationControl.StartPause',...
                'SimulationControl.Step',...
                'SimulationControl.Stop'};
            
            % Add separators only for these
            tagsToSeperators = {'Annotation.InsertLegend','Exploration.ZoomIn'};
            tbh = findall(hfig,'Type','uitoggletool');
            
            for k = 1:numel(tbh)
                tbh(k).Separator = 'off';
                tbh(k).HandleVisibility = 'on';
                if any(strcmp(tbh(k).Tag,tagsToKeep))
                    tbh(k).Visible = 'on';
                    if any(strcmp(tbh(k).Tag,tagsToSeperators))
                        tbh(k).Separator = 'on';
                    end
                else
                    tbh(k).Visible = 'off';
                end
            end
            
            tbh = findall(hfig,'Type','uipushtool');
            for k = 1:numel(tbh)
                tbh(k).Separator = 'off';
                tbh(k).HandleVisibility = 'on';
                if any(strcmp(tbh(k).Tag,tagsToKeep))
                    tbh(k).Visible = 'on';
                    if any(strcmp(tbh(k).Tag,tagsToSeperators))
                        tbh(k).Separator = 'on';
                    end
                else
                    tbh(k).Visible = 'off';
                end
            end
            
            tbh = findall(hfig,'Tag','Exploration.Brushing');
            tbh.Separator = 'off';
            tbh.Visible = 'off';
            tbh.HandleVisibility = 'on';
        end
        
        function reset(obj)
            allIcons = getIcons;
            obj.StartIcon = allIcons.run;
            obj.PauseIcon = allIcons.pause;
            obj.StepForwardIcon = allIcons.next_step;
            obj.StopIcon = allIcons.stop;
            
            % Create uipushtools in the toolbar
            obj.StartPausePushtool = uipushtool(...
                'Parent', obj.Toolbar,...
                'TooltipString',getLabel('Simulink:studio:StartPauseContinueStart'),...
                'CData', obj.StartIcon,...
                'ClickedCallback',@startPausePushtoolCallback,...
                'Tag', 'SimulationControl.StartPause');
            
            obj.StepForwardPushtool = uipushtool(...
                'Parent', obj.Toolbar,...
                'TooltipString',getLabel('Simulink:studio:SimulationForward'),...
                'CData', obj.StepForwardIcon,...
                'ClickedCallback',@stepForwardPushtoolCallback,...
                'Tag', 'SimulationControl.Step');
            
            obj.StopPushtool = uipushtool(...
                'Parent', obj.Toolbar,...
                'TooltipString',getLabel('Simulink:studio:Stop'),...
                'CData', obj.StopIcon,...
                'ClickedCallback',@stopPushtoolCallback,...
                'Tag', 'SimulationControl.Stop');
            
            % Reorder toolbar
            children = get(obj.Toolbar,'Children');
            set(obj.Toolbar,'Children',[children(4:end);children(1:3)]);
            for i=1:numel(children)
                set(children(1),'HandleVisibility','off');
            end
            
            function startPausePushtoolCallback(src,event) %#ok<INUSD>
                simStatus = get_param(obj.ModelName,'SimulationStatus');
                if strcmp(simStatus,'running')
                    set_param(obj.ModelName,'SimulationCommand','Pause');
                    setPausedIcon(obj);
                    obj.StartPauseClicked = true;
                elseif strcmp(simStatus,'paused')
                    set_param(obj.ModelName,'SimulationCommand','Continue');
                    setRunningIcon(obj);
                    obj.StartPauseClicked = true;
                else
                    set_param(obj.ModelName,'SimulationCommand','Start');
                end
            end
            
            function stepForwardPushtoolCallback(src,event) %#ok<INUSD>
                simStatus = get_param(obj.ModelName,'SimulationStatus');
                if ~strcmp(simStatus,'paused')
                    % Put simulation into a paused state
                    set_param(obj.ModelName,'SimulationCommand','Start');
                    set_param(obj.ModelName,'SimulationCommand','Pause');
                end
                set_param(obj.ModelName,'SimulationCommand','step');
            end
            
            function stopPushtoolCallback(src,event) %#ok<INUSD>
                set_param(obj.ModelName,'SimulationCommand','stop');
            end
            
        end
        
        function update(obj)
            if ~isvarname(obj.ModelName) || ~bdIsLoaded(obj.ModelName)
                warning('IMULINKUITOOLBAR:ModelNotLoaded',...
                    'Model is not currently loaded')
                return;
            end
            
            if obj.StartPauseClicked
                obj.StartPauseClicked = false;
                return
            end
            
            simStatus = get_param(obj.ModelName,'SimulationStatus');
            if strcmp(simStatus,'running')
                setRunningIcon(obj);
            elseif strcmp(simStatus,'paused')
                setPausedIcon(obj);
            else
                % Not running
                setStoppedIcon(obj);
            end
        end
        
        function setRunningIcon(obj)
            set(obj.StartPausePushtool,...
                'CData', obj.PauseIcon,...
                'TooltipString',getLabel('Simulink:studio:StartPauseContinuePause'));
            set(obj.StepForwardPushtool,'Enable', 'off');
            set(obj.StopPushtool,'Enable','on');
            drawnow limitrate
        end
        
        function setPausedIcon(obj)
            set(obj.StartPausePushtool,...
                'CData', obj.StartIcon,...
                'TooltipString',getLabel('Simulink:studio:StartPauseContinueContinue'));
            set(obj.StepForwardPushtool,'Enable','on');
            set(obj.StopPushtool,'Enable','on');
            drawnow limitrate
        end
        
        function setStoppedIcon(obj)
            set(obj.StartPausePushtool,...
                'CData', obj.StartIcon,...
                'TooltipString',getLabel('Simulink:studio:StartPauseContinueStart'));
            set(obj.StepForwardPushtool,'Enable','on');
            set(obj.StopPushtool,'Enable','off');
            drawnow limitrate
        end
        
        function addCallbackFunctionToParameter(obj, paramName, cbFcn)
            % Get model name
            modelName = obj.ModelName;
            blockName = obj.BlockName;
            
            % Keep model dirty flag
            isModelDirty = get_param(modelName,'Dirty');
            
            % Get current callback functions
            cbs = get_param(blockName,paramName);
            
            % Add callback function to paramName
            if isempty(cbs)
                cbs = cbFcn;
            else
                cbs = sprintf('%s\n%s',cbs,cbFcn);
            end
            set_param(blockName,paramName,cbs)
            
            % Return model's dirty flag to previous state
            set_param(modelName,'Dirty',isModelDirty)
        end
        
        function removeCallbackFunctionFromParameter(obj, paramName, cbFcn)
            % Get model name
            modelName = obj.ModelName;
            blockName = obj.BlockName;
            
            % Keep model dirty flag
            isModelDirty = get_param(modelName,'Dirty');
            
            % Get current callback functions
            cbs = get_param(blockName,paramName);
            
            % Remove cbFcn from the current list
            cbs = strrep(cbs,cbFcn,'');
            set_param(blockName,paramName,cbs)
            
            % Return model's dirty flag to previous state
            set_param(modelName,'Dirty',isModelDirty)
        end
    end
end

function icons = getIcons
icons = load('uiscope_icons.mat');
end

function str = getLabel(msg)
str = getString(message(msg));
str = strrep(str,'&',''); %gets rid of '&'
end