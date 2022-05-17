classdef  (StrictDefaults)HelperUpdatePose < ...
        matlabshared.tracking.internal.SimulinkBusUtilities
    
    % This is a helper block for example purposes and may be removed or
    % modified in the future.
    %
    % Copyright 2019-2021 The MathWorks, Inc.
    
    properties(Constant, Access=protected)
        pBusPrefix = 'BusUpdatePose'
    end
    
    properties (Access = private)
        %Logged Data
        pData
        
        %CurrentIndex
        pCurrentIndex = 1
    end
    
    properties(Nontunable)
        %DataFile Enter data file name
        DataFile = {'StateParamV1'};
        
        %DataVariableName Name of the Vehicle data in
        DataVariableName = 'dataV1';
        
        %TrackerId Tracker Index
        TrackerId = 1;
    end
    
    methods
        function obj = HelperUpdatePose(varargin)
            % Constructor
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access=protected)
        
        function [out, argsToBus] = defaultOutput(obj)
            stateParams = struct('Frame',fusionCoordinateFrameType(1),'Position',[0 0 0],'Velocity',[0 0 0]);
            OT = objectTrack('TrackLogicState',false(1,3),'StateParameters',stateParams);
            st = toStruct(OT);
            if strcmpi(st.TrackLogic,'History')
                st.TrackLogic = trackLogicType(1);
            elseif strcmpi(st.TrackLogic,'Score')
                st.TrackLogic = trackLogicType(2);
            else
                st.TrackLogic = trackLogicType(3);
            end
            st.TrackLogicState = false(1,3);
            trk = optionalObjectAttributes(obj,st);
            track = optionalStateParameters(obj,trk);
            track.StateSize = uint32(0);
            track.LogicSize = uint32(0);
            out = track;
            argsToBus = {out};
        end
        
        % Currently only support simulink environment, so nothing to do
        % here
        function tracksOutput = sendToBus(obj, st, varargin) %#ok<*INUSL>
            % sampleData can have the information of sample tracks or the
            % tracker info depending upon the busIdx.
            sampleData = varargin{1};
            fullTracksList = repmat(sampleData, [200, 1]);
            tracksOutput = struct('NumTracks', 200, 'Tracks', fullTracksList);
        end
        
        function setupImpl(obj, varargin)
            val = load([obj.DataFile{1},'.mat']);
            obj.pData = val.(obj.DataVariableName);
        end
        
        function [updatedTracks] = stepImpl(obj, tracks)
            if strcmpi(obj.BusName,'UpdatePoseV1')
                data = get_param([gcs,'/','JPDA Tracker V1'],'RunTimeObject');
            else
                data = get_param([gcs,'/','JPDA Tracker V2'],'RunTimeObject');
            end
            if tracks.NumTracks>0
            numTracks = data.OutputPort(1).Data.NumTracks;
            if numTracks>0
                trs = tracks.Tracks(1:numTracks);
                if isfield(trs,'ObjectAttributes')
                    trs = rmfield(trs,'ObjectAttributes');
                end
                if isfield(tracks.Tracks,'ObjectAttributes')
                    tracks.Tracks = rmfield(tracks.Tracks,'ObjectAttributes');
                end
                updateTrs = getUpdatedTracksParam(obj, trs);
                updatedTracks = getUpdatedTracks(obj, updateTrs, tracks);
            else
               if isfield(tracks.Tracks,'ObjectAttributes')
                    tracks.Tracks = rmfield(tracks.Tracks,'ObjectAttributes');
                end
                updatedTracks = tracks;
            end
            else
                if strcmpi(obj.BusName ,'UpdatePoseV1')
                    val = defaultOutput(obj);
                else
                    val = defaultOutput(obj);
                    val.SourceIndex = uint32(2);
                end
                updatedTracks = sendToBus(obj, [], val);
                updatedTracks.NumTracks = 0;
            end
            obj.pCurrentIndex = obj.pCurrentIndex+1;
        end
        
        function updateTrs = getUpdatedTracksParam(obj, t)
            for i = 1:numel(t)
                t(i).SourceIndex = uint32(obj.TrackerId);
                if ~isempty(obj.pData{obj.pCurrentIndex})                
                    t(i).StateParameters.Frame = fusionCoordinateFrameType(1);
                    t(i).StateParameters.Position = obj.pData{obj.pCurrentIndex}.Position;
                    t(i).StateParameters.Velocity = obj.pData{obj.pCurrentIndex}.Velocity;
                else
                    t(i).StateParameters.Frame = fusionCoordinateFrameType(1);
                    t(i).StateParameters.Position = [0 0 0];
                    t(i).StateParameters.Velocity = [0 0 0];
                end
            end
            updateTrs  = t;
        end
        
        function out = getUpdatedTracks(~, updateTrs,tracks)
            numTracks = numel(updateTrs);
            for i = 1:numTracks
                tracks.Tracks(i) =  updateTrs(i);
            end
            out = tracks;
        end
        
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        
        function status = isEndOfData(obj)
            % Return true if end of data has been reached
            status = (obj.pCurrentIndex == obj.pMaxIndex);
        end
        
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);
        end
        
        function dt = getOutputDataTypeImpl(obj)
            dt = getOutputDataTypeImpl@matlabshared.tracking.internal.SimulinkBusUtilities(obj);
        end
        
        function str = getIconImpl(~)
            str = sprintf('Helper\nUpdatePose');
        end
        
        function varargout = getOutputNamesImpl(~)
            varargout{1} = sprintf('Tracks');
        end
        function varargout = getInputNamesImpl(~)
            varargout{1} = sprintf('In');
        end
        function [out1,out2] = isOutputFixedSizeImpl(~)
            out1 = true;
            out2 = true;
        end
        function [out1,out2] = getOutputSizeImpl(~)
            out1 = [1 1];
            out2 = [1 1];
        end
        
        function [out1,out2] = isOutputComplexImpl(~)
            out1 = false;
            out2 = false;
        end
    end
    
    methods(Access=protected)
        function stOut = optionalObjectAttributes(~,stIn)
            % Removes ObjectAttributes from output when
            % they are not provided in the input detections.
            
            hasAttribs = false;
            if isstruct(stIn) && isfield(stIn,'ObjectAttributes') && ~isempty(fieldnames(stIn.ObjectAttributes)) && isstruct(stIn.ObjectAttributes)
                hasAttribs = true;
            end
            if hasAttribs
                stOut = stIn;
            else
                % Remove ObjectAttributes from struct
                if coder.target('MATLAB')
                    if ~hasAttribs && isfield(stIn,'ObjectAttributes')
                        stOut = rmfield(stIn,'ObjectAttributes');
                    else
                        stOut = stIn;
                    end
                else
                    stOut = struct;
                    flds = fieldnames(stIn);
                    for m = 1:numel(flds)
                        thisFld = flds{m};
                        if ~hasAttribs && strcmp(thisFld,'ObjectAttributes')
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
            
            if  ~isequal(stIn.StateParameters,struct)
                stOut = stIn;
            else
                % Remove StateParameters from struct
                if coder.target('MATLAB')
                    if isequal(stIn.StateParameters,struct) && isfield(stIn,'StateParameters')
                        stOut = rmfield(stIn,'StateParameters');
                    else
                        stOut = stIn;
                    end
                else
                    stOut = struct;
                    flds = fieldnames(stIn);
                    for m = 1:numel(flds)
                        thisFld = flds{m};
                        if isequal(stIn.StateParameters,struct) && strcmp(thisFld,'StateParameters')
                            continue
                        end
                        stOut.(thisFld) = stIn.(thisFld);
                    end
                end
            end
        end
    end
    
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = true;
        end
        
    end
    
    methods(Static, Access=protected)
        function groups = getPropertyGroupsImpl
            pList = {'DataFile', 'DataVariableName', 'TrackerId'};
            pSection = matlab.system.display.Section('PropertyList',pList);
            
            slBusSection = getPropertyGroupsImpl@matlabshared.tracking.internal.SimulinkBusUtilities;
            
            groups = [pSection, slBusSection];
        end
        
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title', 'HelperUpdatePose', ...
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
str = sprintf('This block updates the StateParameters and trackerIndex values of tracks propagated from trackerJPDA tracker blocks from vehicle1 and vehicle2.');
end

