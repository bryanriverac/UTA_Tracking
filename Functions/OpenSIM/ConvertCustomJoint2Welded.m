function xml_doc = ConvertCustomJoint2Welded(xml_doc, jointname)

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

        % Clone frames (if present)
        framesList = jointNode2Convert.getElementsByTagName('frames');
        if framesList.getLength > 0
            framesClone = framesList.item(0).cloneNode(true);
        else
            framesClone = [];
        end

        % Create WeldJoint
        weldNode = xml_doc.createElement('WeldJoint');
        weldNode.setAttribute('name', jointname);

        % socket_parent_frame
        spfNode = xml_doc.createElement('socket_parent_frame');
        spfNode.appendChild(xml_doc.createTextNode(socket_parent_frame));
        weldNode.appendChild(spfNode);

        % socket_child_frame
        scfNode = xml_doc.createElement('socket_child_frame');
        scfNode.appendChild(xml_doc.createTextNode(socket_child_frame));
        weldNode.appendChild(scfNode);

        % Append frames if they exist
        if ~isempty(framesClone)
            weldNode.appendChild(framesClone);
        end

        % Replace joint
        objectsNode.replaceChild(weldNode, jointNode2Convert);
        break;
    end
end

% Check if the joint was found
if isempty(jointNode2Convert)
    error('CustomJoint "%s" not found.', jointname);
end
