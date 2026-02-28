function SphereContactSpace = createContactSphere(nameSphere, FrameBody, SphereRadius, SphereContactLocation, SphereContactOrientation)

import org.opensim.modeling.*;


SphereContactSpace = ContactSphere();
SphereContactSpace.setRadius(SphereRadius);
SphereContactSpace.setLocation(SphereContactLocation);
SphereContactSpace.setOrientation(SphereContactOrientation);
SphereContactSpace.setFrame(FrameBody);
SphereContactSpace.setName(nameSphere);


