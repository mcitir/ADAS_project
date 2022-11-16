function manipulatedParameter = manipulate(parameter, manipulationModel)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

if nargin < 2
  manipulationModel = 'random';
end

if isequal(manipulationModel, 'random')
    for i = 1:numel(parameter)
        for k = 1: max(size(parameter{i}.Measurement))
            randomNum = 3+rand(1,1)*(4-3);
            parameter{i}.Measurement(k) = parameter{i}.Measurement(k) + randomNum;
        end
    end
end
% if isequal(manipulationModel, 'static')
%     for i = 1:numel(parameter)
%         for k = 1: max(size(parameter{i}.Measurement))
%             parameter{i}.Measurement(k) = parameter{i}.Measurement(k) + 2;
%         end
%     end
% end


manipulatedParameter = parameter;
end