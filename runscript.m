% clean up
clear variables; clc; close all; dbstop if error;

% add subfolders to path
addpath(genpath('./'));

% fname_list = {'mapTask3_newlandmarks'};%{'mapTask3','mapTask3_1','mapTask3_2','mapTask3_3', 'mapTask3_4', 'mapTask3_5', 'mapTask3_6', 'mapTask3_7'};

fname_list = {'mapSingleHomotopy'}

for i = 1:size(fname_list,2)
    main(fname_list{i});
end
    