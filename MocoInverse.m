%% We will use MocoInverse for the initial guess of static optimization

import org.opensim.modeling.*;

% 1. Setup the Inverse Tool
inverse = MocoInverse();
inverse.setName('MocoInverse_Periodic_Guess');

% 2. Use your existing ModelProcessor
modelProcessor = ModelProcessor(fullfile('Model','GenericAmputee_r_tracking.osim'));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
inverse.setModel(modelProcessor);

% 3. Set Kinematics (MocoInverse tracks these exactly)
ikTable = TableProcessor(fullfile('Data','ik_output_walk_rad.sto'));
ikTable.append(TabOpLowPassFilter(6));
inverse.setKinematics(ikTable);

% 4. Set Time (Matching your new limits)
inverse.set_initial_time(0.81);
inverse.set_final_time(1.95);
inverse.set_mesh_interval(0.02); % Fine mesh for better contact resolution

% 5. Initialize the Study to add the Periodicity Goal
study = inverse.initialize();
problem = study.updProblem();

% 6. Add Periodicity Goal
periodicityGoal = MocoPeriodicityGoal('periodic');
problem.addGoal(periodicityGoal);

% Get model for state/control names
model = modelProcessor.process();
model.initSystem();

% Make all states (except pelvis_tx) periodic
for i = 0:model.getNumStateVariables()-1
    name = char(model.getStateVariableNames().getitem(i));
    if ~contains(name, 'pelvis_tx/value')
        periodicityGoal.addStatePair(MocoPeriodicityGoalPair(name));
    end
end

% Make all controls (muscles/reserves) periodic
for i = 0:model.getNumControls()-1
    name = char(model.getControlNames().getitem(i));
    periodicityGoal.addControlPair(MocoPeriodicityGoalPair(name));
end

% 7. Solve and Save
solutionInverse = study.solve();
solutionInverse.write('Data/periodic_guess_from_inverse.sto');