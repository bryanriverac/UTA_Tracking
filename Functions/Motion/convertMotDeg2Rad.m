function MotDataRad = convertMotDeg2Rad(MotionData)
%CONVERTMOTDEG2RAD Convert motion data from degrees to radians, use this
%function only for IK data. 
%   MotDataRad = convertMotDeg2Rad(MotionData)
%
%   - MotionData: struct from read_motionFile.m
%   - Uses MotionData.labels to identify columns
%   - Column 1 (time) is NOT converted
%   - Columns with labels 'pelvis_tx', 'pelvis_ty', 'pelvis_tz'
%     are NOT converted (translations in m)
%   - All other columns are converted deg → rad, including pelvis_tilt,
%     pelvis_list, pelvis_rotation, etc.

    MotDataRad = MotionData;

    data   = MotionData.data;
    labels = MotionData.labels;

    % Safety check
    if size(data,2) ~= numel(labels)
        error('convertMotDeg2Rad:LabelMismatch', ...
              'Number of labels does not match number of data columns.');
    end

    % Define pelvis translation labels to skip (lowercase for robustness)
    skipLabels = {'pelvis_tx','pelvis_ty','pelvis_tz'};

    % Loop over columns (skip time explicitly)
    for i = 2:numel(labels)
        lbl = lower(strtrim(labels{i}));

        % Skip pelvis translations
        if any(strcmp(lbl, skipLabels))
            continue
        end

        % Convert degrees → radians
        data(:,i) = deg2rad(data(:,i));
    end

    MotDataRad.data = data;
end