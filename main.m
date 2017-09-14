%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo for a 2D belief space planning scenario with a
% point robot whose body is modeled as a disk
% and it can sense beacons in the world.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function main(fname)
% add subfolders to path
% addpath(genpath('./'));

% clean up
% clear variables; clc; close all; dbstop if error;

fprintf('\n A demonstration of the iLQG algorithm for Belief Space Planning \n')

% Whether or not to store data to file
CREATE_OUTPUT_DIRECTORY = 1; % set to 1 for writing output

% number of sims to run
NUM_SIMS = 1;

% which map to use
% fname = 'mapTask3';

% create full path to map name
mapFilePath = strcat('./Maps/',fname,'.mat');

time = clock; % Gets the current time as a 6 element vector

newFolderName = [fname,'_',...
    num2str(time(1)),'_',... % Returns year as character
    num2str(time(2)),'_',... % Returns month as character
    num2str(time(3)),'_',... % Returns day as char
    num2str(time(4)),'_',... % resturns hour as char..
    num2str(time(5))]; %returns minute as char     

% Lets check for platform type i.e., Windows, Linux and define base folder
% accordingly base diretory where runs live
if isunix ==1
    [~,username] = system('whoami');
    baseDirectory = ['/home/',username(1:end-1),'/MATLAB/TRO/SingleHomotopy/'];
    % Mac is unix so have to check here
    if ismac==1
        baseDirectory = ['/Users/',username(1:end-1),'/Documents/MATLAB/TRO/SingleHomotopy/'];
    end
end

% Create new directory
if CREATE_OUTPUT_DIRECTORY
    fstat = mkdir(baseDirectory,newFolderName);
    
    % if unsuccessful, exit
    if fstat == 0
        error('Could not create directory to save files');
    end
    
end

% path to where data is written
outDatPath = strcat(baseDirectory,newFolderName,'/');

for i = 1:NUM_SIMS
    
    if CREATE_OUTPUT_DIRECTORY
        mkdir(outDatPath,['run',num2str(i)]);
    end
    
    plan_2dpointrobot(mapFilePath, [outDatPath,'run',num2str(i),'/']);
end
end