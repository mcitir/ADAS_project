function [radarDetections, ptCloud1,ptCloud2, egoPose] = helperCollectSensorData(egoVehicle, radars, lidar1,lidar2, time)

% Current poses of targets with respect to ego vehicle
tgtPoses = targetPoses(egoVehicle);
    
radarDetections = cell(0,1);
for i = 1:numel(radars)
    thisRadarDetections = step(radars{i},tgtPoses,time);
    radarDetections = [radarDetections;thisRadarDetections]; %#ok<AGROW>
end

% Generate point cloud from lidar
rdMesh = roadMesh(egoVehicle);
ptCloud1 = step(lidar1, tgtPoses, rdMesh, time);
ptCloud2 = step(lidar2, tgtPoses, rdMesh, time);

lidar2_=lidar2;
% Compute pose of ego vehicle to track in scenario frame. Typically
% obtained using an INS system. If unavailable, this can be set to
% "origin" to track in ego vehicle's frame.
egoPose = pose(egoVehicle);

end