% This function generates the GRF solution from a MocoTrack Solution 

clc; clear; close all;
import org.opensim.modeling.*;
orgDir  = pwd; 
cd(getDataPath());
%%  1. Setup the tool


modelProcessor = ModelProcessor(fullfile('Model','GenericAmputee_r_tracking.osim'));

% Adapt it to the model that was simulated 
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));

model = modelProcessor.process();
model.initSystem();


solution = MocoTrajectory(fullfile('Results','MTracking_Amp_STF_1_solution.sto'));
statesTable = solution.exportToStatesTable();

% Convert to StatesTrajectory
statesTraj = StatesTrajectory.createFromStatesTable(model, statesTable);
nStates = statesTraj.getSize();


%% 

% --- Contact sphere names ---
contact_r = {'contactHeel_r','contactMH1_r','contactMH3_r',...
             'contactMH5_r','contactOtherToes_r','contactHallux_r'};

contact_l = {'contactHeel_l','contactMH1_l','contactMH3_l',...
             'contactMH5_l','contactOtherToes_l','contactHallux_l'};

time = zeros(nStates,1);

GRF_r = zeros(nStates,3);
GRF_l = zeros(nStates,3);

COP_r = zeros(nStates,3);
COP_l = zeros(nStates,3);

FreeMoment_r = zeros(nStates,1);
FreeMoment_l = zeros(nStates,1);

% ==========================================================
%                     MAIN LOOP
% ==========================================================
for i = 0:nStates-1
    
    state = statesTraj.get(i);
    model.realizeDynamics(state);
    
    idx = i + 1;
    time(idx) = state.getTime();
    
    % ---------- RIGHT FOOT ----------
    totalF_r = zeros(1,3);
    totalM_r = zeros(1,3);
    
    for j = 1:length(contact_r)
        comp = SmoothSphereHalfSpaceForce.safeDownCast( ...
            model.getComponent(['/forceset/' contact_r{j}]));
        
        vec = comp.getRecordValues(state);
        
        F = [vec.get(0), vec.get(1), vec.get(2)];
        M = [vec.get(3), vec.get(4), vec.get(5)];
        
        totalF_r = totalF_r + F;
        totalM_r = totalM_r + M;
    end
    
    GRF_r(idx,:) = totalF_r;
    
    Fy_r = totalF_r(2); % vertical force
    
    if abs(Fy_r) > 1e-5
        COPx = - totalM_r(3) / Fy_r;
        COPz =   totalM_r(1) / Fy_r;
        
        COP_r(idx,:) = [COPx 0 COPz];
        
        FreeMoment_r(idx) = totalM_r(2) ...
            - (COPx * totalF_r(3) - COPz * totalF_r(1));
    else
        COP_r(idx,:) = [NaN NaN NaN];
        FreeMoment_r(idx) = NaN;
    end
    
    
    % ---------- LEFT FOOT ----------
    totalF_l = zeros(1,3);
    totalM_l = zeros(1,3);
    
    for j = 1:length(contact_l)
        comp = SmoothSphereHalfSpaceForce.safeDownCast( ...
            model.getComponent(['/forceset/' contact_l{j}]));
        
        vec = comp.getRecordValues(state);
        
        F = [vec.get(0), vec.get(1), vec.get(2)];
        M = [vec.get(3), vec.get(4), vec.get(5)];
        
        totalF_l = totalF_l + F;
        totalM_l = totalM_l + M;
    end
    
    GRF_l(idx,:) = totalF_l;
    
    Fy_l = totalF_l(2);
    
    if abs(Fy_l) > 1e-5
        COPx = - totalM_l(3) / Fy_l;
        COPz =   totalM_l(1) / Fy_l;
        
        COP_l(idx,:) = [COPx 0 COPz];
        
        FreeMoment_l(idx) = totalM_l(2) ...
            - (COPx * totalF_l(3) - COPz * totalF_l(1));
    else
        COP_l(idx,:) = [NaN NaN NaN];
        FreeMoment_l(idx) = NaN;
    end
    
end