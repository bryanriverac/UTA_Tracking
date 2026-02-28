clear; close all; clc

addpath(fullfile('Functions','OpenSIM'));
orgDir  = pwd; 

%% 
% Load model
model_name = 'GenericAmputee_r.osim';
AmpLimb = 'r'; % tell the amputated side


% Weld the joints f
ReduceJointsModel(model_name, AmpLimb);

%%  Now add the elements for tracking simulations
import org.opensim.modeling.*;
cd(getDataPath());
wModelName = [model_name(1:end-5), '_welded.osim'];

% Load new model
model = Model(fullfile('Model', wModelName));

model = createContactElement(model, AmpLimb);
% Now create the spring 

model.initSystem();

% Create spring force
springFlexion = SpringGeneralizedForce();
springFlexion.setName(['prosthetic_flexion_', AmpLimb]);

% Attach to coordinate
springFlexion.set_coordinate(['ankle_angle_', AmpLimb]);

% Set parameters
springFlexion.set_stiffness(700);      % Nm/rad
springFlexion.set_rest_length(0);      % rad
springFlexion.set_viscosity(1);        % Nms/rad

% Add force to model
model.addForce(springFlexion);

% Finalize and save
model.finalizeConnections();
model.print(fullfile('Model', [model_name(1:end-5), '_tracking.osim']));

cd(orgDir)



