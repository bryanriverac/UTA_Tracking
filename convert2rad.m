clc; clear; close all;
import org.opensim.modeling.*;

table = TimeSeriesTable('Data/ik_output_walk.mot');

model = Model('Model/GenericAmputee_r_tracking.osim');
model.initSystem();
coordSet = model.getCoordinateSet();

labels = table.getColumnLabels();
nRows = table.getNumRows();

for i = 0:labels.size()-1
    label = string(labels.get(i));

    if coordSet.contains(label)
        coord = coordSet.get(label);
        if strcmp(coord.getMotionType().toString(), 'Rotational')

            col = table.updDependentColumn(label);
            for r = 0:nRows-1
                col.set(r, col.get(r) * pi/180);
            end

        end
    end
end

% Mark data as radians (not degrees)
table.addTableMetaDataString('inDegrees', 'no');

STOFileAdapter.write(table, 'Data/ik_output_walk_rad.sto');