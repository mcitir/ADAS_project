function output = initilizeGospa(scenario)
    output.lgospa       = zeros(1,size(scenario,2));
    output.gospa        = zeros(1,size(scenario,2));
    output.switching    = zeros(1,size(scenario,2));
    output.localization = zeros(1,size(scenario,2));
    output.missTarget   = zeros(1,size(scenario,2));
    output.falseTracks  = zeros(1,size(scenario,2));
end