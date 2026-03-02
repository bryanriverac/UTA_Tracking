%% We will use MocoInverse for the initial guess of static optimization
clc; clear; close all;
orgDir  = pwd; 
import org.opensim.modeling.*;
cd(getDataPath());
% 1. Setup the Inverse Tool
inverse = MocoInverse();
inverse.setName('MocoInverse_Periodic_Guess');

% 2. Use your existing ModelProcessor
modelProcessor = ModelProcessor(fullfile('Model','GenericAmputee_r_inverse.osim'));
modelProcessor.append(ModOpAddExternalLoads('Data/grf_walk.xml'));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
modelProcessor.append(ModOpAddReserves(1.0)); % this adds reserve actuators to the joints that do not have any

inverse.setModel(modelProcessor);

% 3. Set Kinematics (MocoInverse tracks these exactly)
ikTable = TableProcessor(fullfile('Data','ik_output_walk_rad.sto'));
ikTable.append(TabOpLowPassFilter(6));
inverse.setKinematics(ikTable);

% 4. Set Time (Matching your new limits)
inverse.set_initial_time(0.81);
inverse.set_final_time(1.95);
%inverse.set_mesh_interval(0.02); % Fine mesh for better contact resolution

% 5. Initialize the Study to add more setting 
study = inverse.initialize();

solver = MocoCasADiSolver.safeDownCast(study.updSolver());

solver.set_optim_max_iterations(2000);
solver.set_num_mesh_intervals(50);
solver.set_optim_constraint_tolerance(1e-4);
solver.set_optim_convergence_tolerance(1e+01);
solver.set_minimize_implicit_auxiliary_derivatives(true)
solver.set_implicit_auxiliary_derivatives_weight(0.00001)
solver.set_enforce_constraint_derivatives(true)


% Solve and visualize.
solution = study.solve();%.unseal();
solution = solution.unseal();
%import org.opensim.modeling.*;
solution.write(fullfile('Results','MInverse_Amp_STF_1_solution.sto'));

cd(orgDir);
% 
% 
% 
% % Get model for state/control names
% model = modelProcessor.process();
% model.initSystem();
% 
% 
% 
% % 7. Solve and Save
% solutionInverse = study.solve();
% solutionInverse.write('Data/periodic_guess_from_inverse.sto');