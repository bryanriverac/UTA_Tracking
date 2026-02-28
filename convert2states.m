import org.opensim.modeling.*;

oldTable = TimeSeriesTable('Data/ik_output_walk_rad.sto');

model = Model('Model/GenericAmputee_r_tracking.osim');
model.initSystem();
coordSet = model.getCoordinateSet();

newTable = TimeSeriesTable();
newTable.setIndependentColumn(oldTable.getIndependentColumn());

labels = oldTable.getColumnLabels();
nRows = oldTable.getNumRows();

for i = 0:labels.size()-1
    label = string(labels.get(i));
    col = oldTable.getDependentColumn(label);

    if coordSet.contains(label)
        coord = coordSet.get(label);
        statePath = coord.getAbsolutePathString() + "/value";
        newCol = Vector(nRows);
        for r = 0:nRows-1
            newCol.set(r, col.get(r));
        end
        newTable.appendColumn(statePath, newCol);
    else
        % keep non-coordinate columns as-is
        newTable.appendColumn(label, col);
    end
end

newTable.addTableMetaDataString('inDegrees','no');
STOFileAdapter.write(newTable,'Data/ik_output_walk_states.sto');