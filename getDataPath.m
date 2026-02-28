function DataPath = getDataPath()
% getDataPath  Return the directory of the data

if ispc
    % Windows: edit this path to where the dataset is stored locally
    DataPath = 'C:\Users\brive\OneDrive - Queen''s University\Research projects\Dataset imperial\Dataset';

elseif ismac
    % macOS: edit this path to where the dataset is stored locally
    DataPath = '/Users/bryanrivera/Library/CloudStorage/OneDrive-Queen''sUniversity/Research projects/MocoTracking';
end