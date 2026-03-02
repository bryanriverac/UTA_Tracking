clc; clear; close all; 
orgDir  = pwd; 

addpath(fullfile('Functions','Motion'));
cd(getDataPath());
mass = 73.49; 

%% 
Data2Compare = 'GRF'; % it can be GRF or IK 

if strcmp(Data2Compare, 'GRF')
    namesolution = 'MTracking_Amp_STF_1_GRF.sto';
    nameReference = 'grf_walk.mot';
    a = 3; 
    b = 3; 
elseif strcmp(Data2Compare, 'IK')
    namesolution = 'MTracking_Amp_STF_1_solution.sto';
    nameReference = 'ik_output_walk_rad.sto';
    a = 3; 
    b = 4; 
end

ReferenceDir = fullfile('Data', nameReference);
ResultsDir = fullfile('Results', namesolution);

dataReference = read_motionFile(ReferenceDir);
dataResults = importdata(ResultsDir); % read_motionFile does not work with MocoTrack results 
% files, so we use this function instead

labels2compare = dataReference.labels;
labelsResults = dataResults.colheaders;
time_ref = dataReference.data(:,1);
time_rslt = dataResults.data(:,1);

for i = 2:numel(labels2compare) % We start from 2 as we don't count time
    label = labels2compare{i};
    idx_result = find(strcmp(labelsResults, label));

    coordReference = dataReference.data(:,i);
    coordResult = dataResults.data(:,idx_result);

    p = mod(i-1, a*b); % plot index 
    if p == 0
        p = a*b;
    elseif p == 1
        figure;
    end
    if strcmp(Data2Compare, 'IK')
        parts = split(label, '/');
        coordName = parts{end-1};
    else
        coordName = label; % Assign label to coordName for GRF case
    end
    factor = 1; 
    if strcmp(Data2Compare, 'IK')
        if ~(endsWith(coordName, '_tx') || endsWith(coordName, '_ty') ...
                || endsWith(coordName, '_tz'))
            factor = 180/pi;   % Convert radians to degrees for plotting
        end
    elseif strcmp(Data2Compare, 'GRF')
        if contains(coordName, 'force')
            factor = 1/(mass*9.81); % normalize to body weight
        elseif contains(coordName, 'torque')
            factor = 1/mass; % normalize to body mass
        end
    end
    subplot(a,b, p); plot(time_ref, coordReference*factor); hold on;
    plot(time_rslt, coordResult*factor);


    title(coordName, 'Interpreter','none');
    xlabel('Time (s)');
    xlim([time_rslt(1), time_rslt(end)])
    if strcmp(Data2Compare, 'IK')
    if factor == 1
        ylabel('meters(m)');
    else
        ylabel('Degrees (°)')
    end
    elseif strcmp(Data2Compare, 'GRF')
        if contains(coordName, 'force')
            if strcmp(coordName(end-2:end-1), '_v')
                ylabel('Force (BW)')
            elseif strcmp(coordName(end-2:end-1), '_p')
                ylabel('Distance (m)')
            end
        elseif contains(coordName, 'torque')
            ylabel('Moment (N-m/kg)')
        end
    end
    if p == a*b 
        legend('Reference', 'Result');
    end
    
    hold off;


end

cd(orgDir);







