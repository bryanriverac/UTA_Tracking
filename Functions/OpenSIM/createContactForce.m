function contactForce = createContactForce(nameForce, Socket_sphere, Socket_half_space, MiscParameters)

import org.opensim.modeling.*;

contactForce = SmoothSphereHalfSpaceForce();
contactForce.setName(nameForce);


% Connect sockets (THIS IS THE KEY PART)
contactForce.connectSocket_sphere(Socket_sphere);
contactForce.connectSocket_half_space(Socket_half_space);

% Set material/contact properties
contactForce.set_stiffness(MiscParameters.stiffness);
contactForce.set_dissipation(MiscParameters.dissipation);
contactForce.set_static_friction(MiscParameters.static_friction);
contactForce.set_dynamic_friction(MiscParameters.dynamic_friction);
contactForce.set_viscous_friction(MiscParameters.viscous_friction);
contactForce.set_transition_velocity(MiscParameters.transition_velocity);
contactForce.set_hertz_smoothing(MiscParameters.hertz_smoothing);
contactForce.set_hunt_crossley_smoothing(MiscParameters.hunt_crossley_smoothing);