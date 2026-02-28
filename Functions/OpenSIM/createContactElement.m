function model = createContactElement(model, AmpLimb)

import org.opensim.modeling.*;

% Make and add a Platform Body
ground = model.getGround();
% Make a Contact Half Space
groundContactLocation = Vec3(0,0,0);
groundContactOrientation = Vec3(0,0,-1.5708);
groundContactSpace = ContactHalfSpace(groundContactLocation,...
                                       groundContactOrientation,...
                                       ground);
groundContactSpace.setName('floor');

model.addContactGeometry(groundContactSpace);

BodySet = model.getBodySet(); 

% Set the intact limb according to the amputated one 
if strcmp(AmpLimb, 'r')
    IntLimb = 'l';
elseif strcmp(AmpLimb, 'l')
    IntLimb = 'r'; 
else 
    error('Invalid amputated limb specified. Please use ''r'' or ''l''.');
end


% Create the elements in the Prosthetic foot 
foot = BodySet.get(['foot_', AmpLimb]);

% Heel contact
SphereContactLocation = Vec3(0.005,-0.035,0);
SphereContactOrientation = Vec3(0,0,0);
SphereRadius = 0.038;
heel_Amp = createContactSphere(['heel_', AmpLimb], foot, SphereRadius, SphereContactLocation, SphereContactOrientation);

model.addContactGeometry(heel_Amp);

SphereRadius = 0.016;

% Now create the midfoot elements

SphereContactLocation = Vec3(0.13, -0.05, -0.015);
mh1_Amp = createContactSphere(['mh1_', AmpLimb], foot, SphereRadius, SphereContactLocation, SphereContactOrientation);
model.addContactGeometry(mh1_Amp)

SphereContactLocation = Vec3(0.13,-0.05, 0);
mh3_Amp = createContactSphere(['mh3_' , AmpLimb], foot, SphereRadius, SphereContactLocation, SphereContactOrientation);
model.addContactGeometry(mh3_Amp)

SphereContactLocation = Vec3(0.13,-0.05, 0.03);
mh5_Amp = createContactSphere(['mh5_', AmpLimb], foot, SphereRadius, SphereContactLocation, SphereContactOrientation);
model.addContactGeometry(mh5_Amp)

% Now create the forefoot elements
SphereContactLocation = Vec3(0.2, -0.05, -0.01);
hallux_Amp = createContactSphere(['hallux_', AmpLimb], foot, SphereRadius, SphereContactLocation, SphereContactOrientation);
model.addContactGeometry(hallux_Amp)


SphereContactLocation = Vec3(0.2, -0.05, 0.02);
othertoes_Amp = createContactSphere(['othertoes_' , AmpLimb], foot, SphereRadius, SphereContactLocation, SphereContactOrientation);
model.addContactGeometry(othertoes_Amp)

% Create the contact force
ParametersContactForce = struct(); 
ParametersContactForce.stiffness = 306777.6;
ParametersContactForce.dissipation = 2;
ParametersContactForce.static_friction = 0.8;
ParametersContactForce.dynamic_friction = 0.8;
ParametersContactForce.viscous_friction = 0.5;
ParametersContactForce.transition_velocity = 0.2;
ParametersContactForce.hertz_smoothing = 300;
ParametersContactForce.hunt_crossley_smoothing = 50;


contactOtherToes_Amp= createContactForce(['contactOtherToes_', AmpLimb], othertoes_Amp, groundContactSpace, ParametersContactForce);
model.addForce(contactOtherToes_Amp);

contactHeel_Amp = createContactForce(['contactHeel_', AmpLimb], heel_Amp, groundContactSpace, ParametersContactForce);
model.addForce(contactHeel_Amp);

contactMH1_Amp = createContactForce(['contactMH1_', AmpLimb], mh1_Amp, groundContactSpace, ParametersContactForce);
model.addForce(contactMH1_Amp);

contactMH3_Amp = createContactForce(['contactMH3_', AmpLimb], mh3_Amp, groundContactSpace, ParametersContactForce);
model.addForce(contactMH3_Amp);

contactMH5_Amp = createContactForce(['contactMH5_', AmpLimb], mh5_Amp, groundContactSpace, ParametersContactForce);
model.addForce(contactMH5_Amp);


contactHallux_Amp = createContactForce(['contactHallux_', AmpLimb], hallux_Amp, groundContactSpace, ParametersContactForce);
model.addForce(contactHallux_Amp);


% Create the elements in  the Intact foot 

calcn = BodySet.get(['calcn_', IntLimb]);

% Start with the heel 
SphereContactLocation = Vec3(0.031,-0.01,0);
SphereContactOrientation = Vec3(0,0,0);
SphereRadius = 0.038;
heel_Int = createContactSphere(['heel_', IntLimb], calcn, SphereRadius, SphereContactLocation, SphereContactOrientation);
model.addContactGeometry(heel_Int);


% Create contact spheres for the midfoot
SphereRadius = 0.016;
SphereContactLocation = Vec3(0.195, -0.025, 0.025);
mh1_Int = createContactSphere(['mh1_', IntLimb], calcn,  SphereRadius, SphereContactLocation, SphereContactOrientation);
model.addContactGeometry(mh1_Int);


SphereContactLocation = Vec3(0.19, -0.025, -0.0136);
mh3_Int = createContactSphere(['mh3_', IntLimb], calcn, SphereRadius, SphereContactLocation, SphereContactOrientation);
model.addContactGeometry(mh3_Int);


SphereContactLocation = Vec3(0.16, -0.025, -0.04);
mh5_Int = createContactSphere(['mh5_', IntLimb], calcn, SphereRadius, SphereContactLocation, SphereContactOrientation);
model.addContactGeometry(mh5_Int);
% Create contact spheres for the forefoot

toes = BodySet.get(['toes_', IntLimb]); 

% Create contact spheres for the toes
SphereContactLocation = Vec3(0.06, -0.022, 0.02);
hallux_Int = createContactSphere(['hallux_', IntLimb], toes, SphereRadius, SphereContactLocation, SphereContactOrientation);
model.addContactGeometry(hallux_Int);

SphereContactLocation = Vec3(0.045, -0.022, -0.033);
othertoes_Int = createContactSphere(['othertoes_', IntLimb], toes, SphereRadius, SphereContactLocation, SphereContactOrientation);
model.addContactGeometry(othertoes_Int);


contactOtherToes_Int= createContactForce(['contactOtherToes_', IntLimb], othertoes_Int, groundContactSpace, ParametersContactForce);
model.addForce(contactOtherToes_Int);

contactHeel_Int = createContactForce(['contactHeel_', IntLimb], heel_Int, groundContactSpace, ParametersContactForce);
model.addForce(contactHeel_Int);

contactMH1_Int = createContactForce(['contactMH1_', IntLimb], mh1_Int, groundContactSpace, ParametersContactForce);
model.addForce(contactMH1_Int);

contactMH3_Int = createContactForce(['contactMH3_', IntLimb], mh3_Int, groundContactSpace, ParametersContactForce);
model.addForce(contactMH3_Int);

contactMH5_Int = createContactForce(['contactMH5_', IntLimb], mh5_Int, groundContactSpace, ParametersContactForce);
model.addForce(contactMH5_Int);

contactHallux_Int = createContactForce(['contactHallux_', IntLimb], hallux_Int, groundContactSpace, ParametersContactForce);
model.addForce(contactHallux_Int);
