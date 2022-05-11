
clear
angles = [0 0 pi/2];
rot = eul2rotm(angles, "XYZ");
tr = [0 0 0];
trans = rigid3d(rot,tr);

point = ego_blue_10sec(1).PointClouds{1,1};
% point = pointCloud([0 5 3.5; ...
%                     1 4 3]);


ps=20;
poseTest = [file_name(ps).ActorPoses(1).Position,... % ActorPoses(ActorID=1) for blue vehicle
        deg2rad(file_name(ps).ActorPoses(1).Roll),...
        deg2rad(file_name(ps).ActorPoses(1).Pitch),...
        deg2rad(file_name(ps).ActorPoses(1).Yaw)];

eulerTest = [poseTest(4) poseTest(5) poseTest(6)];
rotEgo2WorldTest = eul2rotm(eulerTest, 'XYZ');
trEgo2WorldTest = [poseTest(1) poseTest(2) poseTest(3)];
tformTest = rigid3d(rotEgo2WorldTest, trEgo2WorldTest);
lidarTest{ps,1} = pctransform(lidarData_new{ps,1},tformTest);
playerTest = pcplayer([-600 600],[-600 600],[-2 10]);
view(playerTest,lidarTest{ps,1})


%%%%
point = pctransform(point,trans);

player = pcplayer([-10 10],[-10 10],[-10 10]);
view(player,point)

point2 = pctransform(point,trans);

player2 = pcplayer([-10 10],[-10 10],[-10 10]);
view(player2,point2)