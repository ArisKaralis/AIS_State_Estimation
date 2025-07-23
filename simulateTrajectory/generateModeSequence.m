function [modeSequence, switchTimes] = generateModeSequence(numSteps, params)
% GENERATEMODESEQUENCE - Creates realistic sequence of vessel motion modes
%
% This function generates a time sequence of motion modes that represents
% how real vessels change their motion patterns during a voyage. Vessels
% don't maintain the same motion type throughout - they alternate between
% steady cruise (CV), speed changes (CA), and course changes (CT).
%
% INPUTS:
%   numSteps - Total number of simulation time steps
%   params   - Contains mode transition probability and minimum durations
%
% OUTPUTS:
%   modeSequence - Cell array of mode strings {'CV','CA','CT'} for each step
%   switchTimes  - Array of time steps where mode transitions occur

modeSequence = cell(numSteps, 1);
switchTimes = [];

% SELECT INITIAL MODE with realistic probabilities (70% CV, 15% CA, 15% CT)
currentMode = selectWeightedMode();
modeDuration = 0;
minDuration = ceil(params.modeTransition.minDuration / params.timeStep);

% Maximum duration for CT mode to prevent loops
if isfield(params.modeTransition, 'maxTurnDuration')
    maxTurnDuration = ceil(params.modeTransition.maxTurnDuration / params.timeStep);
else
    maxTurnDuration = ceil(45 / params.timeStep);  % 45 seconds default
end

% GENERATE MODE SEQUENCE over time
for k = 1:numSteps
    modeSequence{k} = currentMode;
    modeDuration = modeDuration + 1;
    
    % CHECK FOR MODE TRANSITION
    shouldTransition = false;
    
    if modeDuration >= minDuration
        % Standard random transition
        if rand() < params.modeTransition.probability
            shouldTransition = true;
        end
        
        % FORCE TRANSITION OUT OF CT MODE to prevent loops
        if strcmp(currentMode, 'CT') && modeDuration >= maxTurnDuration
            shouldTransition = true;
        end
    end
    
    if shouldTransition
        newMode = selectWeightedMode();
        
        % Ensure we actually change modes
        while strcmp(newMode, currentMode) && strcmp(currentMode, 'CT')
            newMode = selectWeightedMode();
        end
        
        if ~strcmp(newMode, currentMode)
            currentMode = newMode;
            switchTimes = [switchTimes; k];
            modeDuration = 0;
        end
    end
end

end

function mode = selectWeightedMode()
% SELECTWEIGHTEDMODE - Choose motion mode with realistic probabilities
% CV: 70% (steady cruise), CA: 15% (speed changes), CT: 15% (course changes)

    modes = {'CV', 'CA', 'CT'};
    weights = [0.70, 0.15, 0.15];  % Realistic operational probabilities
    
    r = rand();
    cumWeight = 0;
    
    for i = 1:length(weights)
        cumWeight = cumWeight + weights(i);
        if r <= cumWeight
            mode = modes{i};
            return;
        end
    end
    
    mode = 'CV';  % Fallback
end