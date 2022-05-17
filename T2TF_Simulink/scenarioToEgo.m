function trackInEgo = scenarioToEgo(trackInScenario)
% Performs coordinate transformation from scenario to ego coordinates
% trackInScenario has StateParameters defined to transform it from scenario
% coordinates to ego coordinates
% We assume a constant velocity model with state [x;vx;y;vy;z;vz]
if isequal(trackInScenario.SourceIndex,1)||isequal(trackInScenario.SourceIndex,2)
    egoPosInScenario = trackInScenario.StateParameters.Position;
    egoVelInScenario = trackInScenario.StateParameters.Velocity;
    stateInScenario = trackInScenario.State;
    stateShift = [egoPosInScenario(1);egoVelInScenario(1);egoPosInScenario(2);egoVelInScenario(2);egoPosInScenario(3);egoVelInScenario(3)];
    stateInEgo = stateInScenario - stateShift;
    trackInEgo = objectTrack('UpdateTime',trackInScenario.UpdateTime,'State',stateInEgo,'StateCovariance',trackInScenario.StateCovariance,'StateParameters',trackInScenario.StateParameters);
else
    trackInEgo = trackInScenario ;
end
end