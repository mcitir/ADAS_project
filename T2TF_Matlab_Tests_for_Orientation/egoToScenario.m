function detection1InScenario = egoToScenario(actor,detection)
%EGOTOSCENARION Summary of this function goes here
%   Detailed explanation goes here

        poseOfActor = deg2rad([actor.Yaw actor.Pitch actor.Roll]);
        rot = eul2tform(poseOfActor, 'ZYX');
        translation = trvec2tform(actor.Position);
        transformation = translation * rot; 
        temp = transformation * [detection.Measurement;1];
        detection1InScenario = temp(1:3);
end

