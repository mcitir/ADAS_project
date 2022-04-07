function positionList = getPositionAsList(Tracks)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

for i=1:numel(Tracks)
    x = Tracks(i).State(1);
    y = Tracks(i).State(3);
    z = Tracks(i).State(6);
    positionList(i,:) = [x y z]; %#ok<AGROW> 
end

end