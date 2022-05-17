function trackInScenario = egoToScenario(trackInEgo)
% Performs coordinate transformation from ego to scenario coordinates
% trackInEgo has StateParameters defined to transform it from ego
% coordinates to scenario coordinates
% We assume a constant velocity model with state [x;vx;y;vy;z;vz]
if isequal(trackInEgo.SourceIndex,1)||isequal(trackInEgo.SourceIndex,2)
    egoPosInScenario = trackInEgo.StateParameters.Position;
    egoVelInScenario = trackInEgo.StateParameters.Velocity;
    stateInScenario = trackInEgo.State;
    stateShift = [egoPosInScenario(1);egoVelInScenario(1);egoPosInScenario(2);egoVelInScenario(2);egoPosInScenario(3);egoVelInScenario(3)];
    stateInEgo = stateInScenario + stateShift;
    trackInScenario = objectTrack('UpdateTime',trackInEgo.UpdateTime,'State',stateInEgo,'StateCovariance',trackInEgo.StateCovariance,'StateParameters',trackInEgo.StateParameters);
else
    trackInScenario =   trackInEgo;
end
end