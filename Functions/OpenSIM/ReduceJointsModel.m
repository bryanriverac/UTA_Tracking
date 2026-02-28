function ReduceJointsModel(model_name, AmpLimb)

xml_file = fullfile('Model', model_name);
xml_doc = xmlread(xml_file);

% Convert the custom joint to a weld joint
xml_doc = ConvertCustomJoint2Welded(xml_doc, ['interfaceSP_' AmpLimb]);
xml_doc = ConvertCustomJoint2Welded(xml_doc, ['interface_' AmpLimb]);


xml_doc = ConvertCustomJoint2Pin(xml_doc, ['ankle_', AmpLimb]);
wModelName = [model_name(1:end-5), '_welded.osim'];


xmlwrite(fullfile('Model', wModelName), xml_doc);

end