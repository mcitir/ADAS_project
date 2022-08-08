function state = getState(object,nameOfState)
%Get an object and list state as requested

if isstruct(object) || iscell(object)
    for i=1:numel(object)
        temp = object(i);
        if size(temp{1,1}.(nameOfState),1)>size(temp{1,1}.(nameOfState),2)
           state(i,:) = transpose(temp{1,1}.(nameOfState)); %#ok<*AGROW> 
        else
           state(i,:) = temp{1,1}.(nameOfState);
        end
        clear temp;
    end
elseif isa(object, 'objectTrack') && isequal(nameOfState, 'State')
    for  i=1:numel(object)
        state(i,:) = [object(i).(nameOfState)(1) object(i).(nameOfState)(3) object(i).(nameOfState)(5)]; %#ok<*AGROW> 
    end
end

end

