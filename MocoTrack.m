clc; clear; close all;
orgDir  = pwd; 

import org.opensim.modeling.*;

% Use this as reference 
track = MocoTrack();
track.setName('muscle_driven_state_tracking');

cd(getDataPath());
% Construct a ModelProcessor and set it on the tool. The default
% muscles in the model are replaced with optimization-friendly
% DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
% parameters.
modelProcessor = ModelProcessor(fullfile('Model','GenericAmputee_r_tracking.osim'));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
track.setModel(modelProcessor);
% This setting allows extra data columns contained in the states
% reference that don't correspond to model coordinates.
track.set_allow_unused_references(true);

%% 
tableStatesProcessor = TableProcessor(fullfile('Data','ik_output_walk_rad.sto'));
tableStatesProcessor.append(TabOpLowPassFilter(6));
track.setStatesReference(tableStatesProcessor);


% Since there is only coordinate position data in the states references, this
% setting is enabled to fill in the missing coordinate speed data using
% the derivative of splined position data.
track.set_track_reference_position_derivatives(true);
% Initial time, final time, and mesh interval.
track.set_initial_time(0.82); % for amputee 0.75- 
track.set_final_time(1.99);
%track.set_mesh_interval(0.05);


% Construct a TableProcessor of the coordinate data and pass it to the 
% tracking tool. TableProcessors can be used in the same way as
% ModelProcessors by appending TableOperators to modify the base table.
% A TableProcessor with no operators, as we have here, simply returns the
% base table.
stateWeights = MocoWeightSet();
stateWeights.cloneAndAppend(MocoWeight('/jointset/ground_pelvis/pelvis_tilt/value',8));
stateWeights.cloneAndAppend(MocoWeight('/jointset/ground_pelvis/pelvis_list/value',41));
stateWeights.cloneAndAppend(MocoWeight('/jointset/ground_pelvis/pelvis_rotation/value',12));
stateWeights.cloneAndAppend(MocoWeight('/jointset/ground_pelvis/pelvis_tx/value',1));
stateWeights.cloneAndAppend(MocoWeight('/jointset/ground_pelvis/pelvis_ty/value',1));
stateWeights.cloneAndAppend(MocoWeight('/jointset/ground_pelvis/pelvis_tz/value',1));

stateWeights.cloneAndAppend(MocoWeight('/jointset/back/lumbar_extension/value',11));
stateWeights.cloneAndAppend(MocoWeight('/jointset/back/lumbar_bending/value',11));
stateWeights.cloneAndAppend(MocoWeight('/jointset/back/lumbar_rotation/value',11));

stateWeights.cloneAndAppend(MocoWeight('/jointset/acromial_r/arm_flex_r/value',3));
stateWeights.cloneAndAppend(MocoWeight('/jointset/acromial_r/arm_add_r/value',1));
stateWeights.cloneAndAppend(MocoWeight('/jointset/acromial_r/arm_rot_r/value',1));

stateWeights.cloneAndAppend(MocoWeight('/jointset/elbow_r/elbow_flex_r/value',3));
stateWeights.cloneAndAppend(MocoWeight('/jointset/radioulnar_r/pro_sup_r/value',1));


stateWeights.cloneAndAppend(MocoWeight('/jointset/acromial_l/arm_flex_l/value',3));
stateWeights.cloneAndAppend(MocoWeight('/jointset/acromial_l/arm_add_l/value',1));
stateWeights.cloneAndAppend(MocoWeight('/jointset/acromial_l/arm_rot_l/value',1));

stateWeights.cloneAndAppend(MocoWeight('/jointset/elbow_l/elbow_flex_l/value',3));
stateWeights.cloneAndAppend(MocoWeight('/jointset/radioulnar_l/pro_sup_l/value',1));


stateWeights.cloneAndAppend(MocoWeight('/jointset/hip_r/hip_flexion_r/value',7));
stateWeights.cloneAndAppend(MocoWeight('/jointset/hip_r/hip_adduction_r/value',16));
stateWeights.cloneAndAppend(MocoWeight('/jointset/hip_r/hip_rotation_r/value',1));
stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/value',5));
stateWeights.cloneAndAppend(MocoWeight('/jointset/ankle_r/ankle_angle_r/value',2)); % the amputated ankle does not need high weight 


stateWeights.cloneAndAppend(MocoWeight('/jointset/hip_l/hip_flexion_l/value',7));
stateWeights.cloneAndAppend(MocoWeight('/jointset/hip_l/hip_adduction_l/value',16));
stateWeights.cloneAndAppend(MocoWeight('/jointset/hip_l/hip_rotation_l/value',1));
stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_l/knee_angle_l/value',5));
stateWeights.cloneAndAppend(MocoWeight('/jointset/ankle_l/ankle_angle_l/value',8));


track.set_states_weight_set(stateWeights);


% Instead of calling solve(), call initialize() to receive a pre-configured
% MocoStudy object based on the settings above. Use this to customize the
% problem beyond the MocoTrack interface.
study = track.initialize();

% Get a reference to the MocoControlGoal that is added to every MocoTrack
% problem by default.
problem = study.updProblem();

% Define the periodicity goal
%periodicityGoal = MocoPeriodicityGoal('symmetryGoal');
%problem.addGoal(periodicityGoal);

model = modelProcessor.process();
model.initSystem();

% % All states are periodic except pelvis anterior-posterior translation
% for i = 1:model.getNumStateVariables()
%     currentStateName = string(model.getStateVariableNames().getitem(i-1));
%     if (~contains(currentStateName,'pelvis_tx/value'))
%        periodicityGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
%     end
% end
% 
% % All controls are periodic
% for i = 1:model.getNumControls()
%     currentControlName = string(problem.createRep().createControlInfoNames().get(i-1));
%     periodicityGoal.addControlPair(MocoPeriodicityGoalPair(currentControlName));
% end

effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
effort.setWeight(1);
effort.setExponent(2);
effort.setDivideByDisplacement(false);

% GRF tracking
contactTracking = MocoContactTrackingGoal('contact', 1);
contactTracking.setExternalLoadsFile('Data/grf_walk.xml');

forceNamesRightFoot = StdVectorString();
forceNamesRightFoot.add('/forceset/contactHeel_r');
forceNamesRightFoot.add('/forceset/contactMH1_r');
forceNamesRightFoot.add('/forceset/contactMH3_r');
forceNamesRightFoot.add('/forceset/contactMH5_r');
forceNamesRightFoot.add('/forceset/contactOtherToes_r');
forceNamesRightFoot.add('/forceset/contactHallux_r');
trackRightGRF = MocoContactTrackingGoalGroup(forceNamesRightFoot,'Right');
%trackRightGRF.append_alternative_frame_paths('/bodyset/toes_r');
contactTracking.addContactGroup(trackRightGRF);

forceNamesLeftFoot = StdVectorString();
forceNamesLeftFoot.add('/forceset/contactHeel_l');
forceNamesLeftFoot.add('/forceset/contactMH1_l');
forceNamesLeftFoot.add('/forceset/contactMH3_l');
forceNamesLeftFoot.add('/forceset/contactMH5_l');
forceNamesLeftFoot.add('/forceset/contactOtherToes_l');
forceNamesLeftFoot.add('/forceset/contactHallux_l');
trackLeftGRF = MocoContactTrackingGoalGroup(forceNamesLeftFoot,'Left');
trackLeftGRF.append_alternative_frame_paths('/bodyset/toes_l');
contactTracking.addContactGroup(trackLeftGRF);

problem.addGoal(contactTracking);

% We might need to modify the code so it adjust the limits according to the
% values of the ik data. 

problem.setStateInfo('/jointset/ground_pelvis/pelvis_tilt/value', [-10*pi/180, 10*pi/180]);
problem.setStateInfo('/jointset/ground_pelvis/pelvis_list/value', [-10*pi/180, 10*pi/180]);
problem.setStateInfo('/jointset/ground_pelvis/pelvis_rotation/value', [-10*pi/180, 10*pi/180]);
problem.setStateInfo('/jointset/ground_pelvis/pelvis_tx/value', [0, 2]); % different than treadmill as the surface does not move
problem.setStateInfo('/jointset/ground_pelvis/pelvis_ty/value', [0.75, 1.25]);
problem.setStateInfo('/jointset/ground_pelvis/pelvis_tz/value', [-0.5, 0.25]);
problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value', [-40*pi/180, 50*pi/180]);
problem.setStateInfo('/jointset/hip_l/hip_adduction_l/value', [-20*pi/180, 20*pi/180]);
problem.setStateInfo('/jointset/hip_l/hip_rotation_l/value', [-40*pi/180, 40*pi/180]);

problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-40*pi/180, 50*pi/180]);
problem.setStateInfo('/jointset/hip_r/hip_adduction_r/value', [-20*pi/180, 20*pi/180]);
problem.setStateInfo('/jointset/hip_r/hip_rotation_r/value', [-40*pi/180, 40*pi/180]);

problem.setStateInfo('/jointset/walker_knee_l/knee_angle_l/value', [0, 75*pi/180]);
problem.setStateInfo('/jointset/walker_knee_r/knee_angle_r/value', [0, 75*pi/180]);

problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value', [-30*pi/180, 50*pi/180]);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [-30*pi/180, 50*pi/180]);

% Put a large weight on the pelvis CoordinateActuators, which act as the
% residual, or 'hand-of-god', forces which we would like to keep as small
% as possible.


% Prescribed average walking speed
%speedGoal = MocoAverageSpeedGoal('speed');
%speedGoal.set_desired_average_speed(AvgSpeed);
%problem.addGoal(speedGoal);

% Prevent body penetration
distanceConstraint = MocoFrameDistanceConstraint();
distanceConstraint.setName('distance_constraint');
distanceConstraint.addFramePair(MocoFrameDistanceConstraintPair('/bodyset/calcn_l','/bodyset/foot_r',0.02,100));
distanceConstraint.addFramePair(MocoFrameDistanceConstraintPair('/bodyset/toes_l', '/bodyset/foot_r', 0.02,100));
problem.addPathConstraint(distanceConstraint);


% Initial guess, comment if there is not guess
solver = MocoCasADiSolver.safeDownCast(study.updSolver());

% Set the normalized tendon forces if not loading initial guess from file
%guess = solver.createGuess();
%numRows = guess.getNumTimes();
%StateNames = model.getStateVariableNames();
%for i = 1:model.getNumStateVariables()
%    currentStateName = string(StateNames.getitem(i-1));
%    if contains(currentStateName,'normalized_tendon_force')
%        MusName = currentStateName;
%        guess.setState(currentStateName, linspace(0.2,0.2,numRows));
%    end
%end
  
% Uncomment this line if not loading an initial guess
%solver.setGuess(guess);

%solver.setGuessFile('Data/MTracking_Amp_1_guess.sto')
solver.set_optim_max_iterations(2000);
solver.set_num_mesh_intervals(50);
solver.set_optim_constraint_tolerance(1e-4);
solver.set_optim_convergence_tolerance(1e-3);
solver.set_minimize_implicit_auxiliary_derivatives(true)
solver.set_implicit_auxiliary_derivatives_weight(0.00001)
solver.set_enforce_constraint_derivatives(true)
%solver.set_lagrange_multiplier_weight(100.0);
solver.resetProblem(problem);

% Solve and visualize.
solution = study.solve();%.unseal();
solution = solution.unseal();
%import org.opensim.modeling.*;
solution.write(fullfile('Results','MTracking_Amp_STF_1_solution.sto'));

contact_r = StdVectorString();
contact_l = StdVectorString();
contact_r.add('/forceset/contactHeel_r');
contact_r.add('/forceset/contactMH1_r');
contact_r.add('/forceset/contactMH3_r');
contact_r.add('/forceset/contactMH5_r');
contact_r.add('/forceset/contactOtherToes_r');
contact_r.add('/forceset/contactHallux_r');

contact_l.add('/forceset/contactHeel_l');
contact_l.add('/forceset/contactMH1_l');
contact_l.add('/forceset/contactMH3_l');
contact_l.add('/forceset/contactMH5_l');
contact_l.add('/forceset/contactOtherToes_l');
contact_l.add('/forceset/contactHallux_l');
externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(model,solution,contact_r,contact_l);

STOFileAdapter.write(externalForcesTableFlat,fullfile('Results','MTracking_Amp_STF_1_GRF.sto'));

model = modelProcessor.process();
report = osimMocoTrajectoryReport(model, ...
        fullfile('Results','MTracking_Amp_STF_1_solution.sto'), 'bilateral', true);
% The report is saved to the working directory.
reportFilepath = report.generate();
%open(reportFilepath);
% study.visualize(solution);

cd(orgDir);