function trackInScenario = egoToScenario2(trackInEgo, agent)
poseOfActor = deg2rad([agent.Actor.Yaw agent.Actor.Pitch agent.Actor.Roll]);
rot = eul2tform(poseOfActor, 'ZYX');
egoPosInScenario = trackInEgo.StateParameters.OriginPosition; % Center of Ego Coordinate Frame
egoVelInScenario = trackInEgo.StateParameters.OriginVelocity; 
translation = trvec2tform(egoPosInScenario);
transformation = translation * rot;
trackInEgoF = [trackInEgo.State(1);trackInEgo.State(3);trackInEgo.State(5)]; % Track in Ego Coordinate Frame
trackInGlobal = transformation * [trackInEgoF;1];
velShift = [trackInEgo.State(2)+egoVelInScenario(1); ...
            trackInEgo.State(4)+egoVelInScenario(2); ...
            trackInEgo.State(6)+egoVelInScenario(3)];
stateInGlob =[trackInGlobal(1);velShift(1);trackInGlobal(2);velShift(2);trackInGlobal(3);velShift(3)];
trackInScenario = objectTrack('UpdateTime',trackInEgo.UpdateTime,'State',stateInGlob,'StateCovariance',trackInEgo.StateCovariance,'StateParameters',trackInEgo.StateParameters);
end

