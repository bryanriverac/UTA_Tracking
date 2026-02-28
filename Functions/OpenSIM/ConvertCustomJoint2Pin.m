function xml_doc = ConvertCustomJoint2Pin(xml_doc, jointname)

% Get Model node
modelNode = xml_doc.getElementsByTagName('Model').item(0);

% Get JointSet
jointSetNode = modelNode.getElementsByTagName('JointSet').item(0);

% Get <objects> inside JointSet
objectsNode = jointSetNode.getElementsByTagName('objects').item(0);

% Get all CustomJoint elements
jointList = objectsNode.getElementsByTagName('CustomJoint');

% Helper to read text of a child tag
readTag = @(node, tag) ...
    char(node.getElementsByTagName(tag).item(0).getFirstChild.getData);

jointNode2Convert = [];

for i = 0:jointList.getLength-1
    jointNode = jointList.item(i);

    if strcmp(char(jointNode.getAttribute('name')), jointname)
        jointNode2Convert = jointNode;

        % Read sockets
        socket_parent_frame = readTag(jointNode2Convert, 'socket_parent_frame');
        socket_child_frame  = readTag(jointNode2Convert, 'socket_child_frame');

        % Clone frames
        framesList = jointNode2Convert.getElementsByTagName('frames');
        if framesList.getLength > 0
            framesClone = framesList.item(0).cloneNode(true);
        else
            framesClone = [];
        end

        % Extract coordinate (keep only ankle_angle_* or first)
        coordSet = jointNode2Convert.getElementsByTagName('coordinates').item(0);
        coordList = coordSet.getElementsByTagName('Coordinate');

        if coordList.getLength == 0
            error('No Coordinate found in CustomJoint "%s".', jointname);
        end

        % Prefer ankle_angle_* if present
        coordNode = [];
        for c = 0:coordList.getLength-1
            tmp = coordList.item(c);
            if contains(char(tmp.getAttribute('name')), 'ankle_angle')
                coordNode = tmp;
                break;
            end
        end

        % Fallback: first coordinate
        if isempty(coordNode)
            coordNode = coordList.item(0);
        end

        coordClone = coordNode.cloneNode(true);

        % Create PinJoint
        pinNode = xml_doc.createElement('PinJoint');
        pinNode.setAttribute('name', jointname);

        % socket_parent_frame
        spfNode = xml_doc.createElement('socket_parent_frame');
        spfNode.appendChild(xml_doc.createTextNode(socket_parent_frame));
        pinNode.appendChild(spfNode);

        % socket_child_frame
        scfNode = xml_doc.createElement('socket_child_frame');
        scfNode.appendChild(xml_doc.createTextNode(socket_child_frame));
        pinNode.appendChild(scfNode);

        % Add coordinates block
        coordsNode = xml_doc.createElement('coordinates');
        coordsNode.appendChild(coordClone);
        pinNode.appendChild(coordsNode);


        % Append frames
        if ~isempty(framesClone)
            pinNode.appendChild(framesClone);
        end

        % Replace joint
        objectsNode.replaceChild(pinNode, jointNode2Convert);
        break;
    end
end

% Check if the joint was found
if isempty(jointNode2Convert)
    error('CustomJoint "%s" not found.', jointname);
end