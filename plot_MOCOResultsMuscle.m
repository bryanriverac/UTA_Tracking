clc; clear; close all;

orgDir  = pwd; 
cd(getDataPath());


%% 
data_type = 'Activation'; 
namesolution = 'MTracking_Amp_STF_1_solution.sto';
ResultsDir = fullfile('Results', namesolution);

dataResults = importdata(ResultsDir);

time_data = dataResults.data(:,1);
labelsResults = dataResults.colheaders;
p = 0;
a = 3;
b = 4;

if strcmp(data_type, 'Force')
    mass = 73.49; 
    import org.opensim.modeling.*;
    mdl = Model(fullfile('Model', 'GenericAmputee_r_tracking.osim'));
    mdl.initSystem()
end
for i = 2:numel(labelsResults)
    label = labelsResults{i};
    plot_index = 0;
    
    if contains(label, '/forceset/')
        parts = split(label, '/');
        normFactor = 1;
        if strcmp(data_type, 'Activation')
            if strcmp(parts{end}, 'activation')
                plot_index = 1;
                muscleName = parts{end-1};
            end
        elseif ~strcmp(parts{end}, 'activation')
            muscleName = parts{end};
            % Get the muscle value for normalization purposes
            if mdl.getMuscles().contains(muscleName)
                plot_index = 1;
                mus = mdl.getMuscles().get(muscleName);
                ForceMuscle = mus.getMaxIsometricForce();
                BW = mass * 9.81;
                normFactor = ForceMuscle / BW;
            end
        end
    end
    if plot_index == 1
        p = p + 1;
        subplot_idx = mod(p, a*b);
        dataMuscle = dataResults.data(:,i)*normFactor;
        if subplot_idx == 0
            subplot_idx = a*b;
        elseif subplot_idx == 1
            figure;
        end
        subplot(a,b, subplot_idx); plot(time_data, dataMuscle ); 
        title(muscleName, 'Interpreter','none');
        xlabel('Time (s)');
        ylabel(data_type);
        if strcmp(data_type, 'Activation')
            ylim([0 1]); % Set y-axis limits for activation plots
        elseif strcmp(data_type, 'Force')
            ylim([0, ceil(max(dataMuscle))]);
        end
    end
end

