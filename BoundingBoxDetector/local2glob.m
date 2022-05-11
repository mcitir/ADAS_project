function points_in_global = local2glob(pose,points) % [x y z roll pitch yaw], [x y z]
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

euler = [deg2rad(pose(4)) deg2rad(pose(5)) deg2rad(pose(6))];
Rotation = eul2tform(euler, 'XYZ');

tr = [pose(1) pose(2) pose(3)];
Translation = trvec2tform(tr);

Transformation = Translation*Rotation;

points_in_global = Transformation * [transpose(points);1];
points_in_global = transpose(points_in_global(1:3));

end