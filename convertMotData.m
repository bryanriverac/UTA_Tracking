clc; clear; close all;


addpath(fullfile('Functions', 'Motion'))
dataIKdeg = read_motionFile(fullfile('Data', 'ik_output_walk.mot'));


dataIKrad = convertMotDeg2Rad(dataIKdeg);


write_motionFile(dataIKrad, fullfile('Data','ik_output_walk_rad.sto'))


