%%%%%%%%%%%%%%%%%%%%%%
% Analyze run data
%%%%%%%%%%%%%%%%%%%%%

baseDirectory = '/home/saurav/MATLAB/TRO/Task3-MoveAway/mapTask3_2017_5_22_12_46/';

% Get list of files in base folder
fileList = dir(baseDirectory);

% Get a logical vector that tells which is a directory.
dirFlags = [fileList.isdir];

% Extract only those that are directories.
subFolders = fileList(dirFlags);

nRuns = 0;

fprintf('Getting list of run folders...  \n');
% get number of folder with the string run in them.
for k = 1 : length(subFolders)
    str = textscan(subFolders(k).name,'%s %*[.]');
    
    idx = regexp(str{1},'run');
    if ~isempty(idx{1})
        nRuns = nRuns + 1;
    end
end

fprintf('Number of runs to analyze: %d \n', nRuns)

nCollisions = 0;

optim_time = zeros(1,nRuns);

optim_time_stddev = 0;

for i=1:nRuns
        
        fprintf('\n \n \n --- BSP-ILQG ANALYSIS OF RUN #%d IN: %s --- \n \n \n',i,baseDirectory)
        
        % the path of this run folder
        runFolder = strcat(baseDirectory,'run',num2str(i),'/');
        
        load(strcat(runFolder,'ilqg_results.mat'),'results');       
        
        if results.collision{1} == 1
            nCollisions = nCollisions + 1;
%             fprintf('Collision in RUN #%d \n',i)
%             openfig(strcat(runFolder,'iLQG-A2B-solution.fig'));
        end
        
        optim_time(i) = results.time{1};
                        
end

fprintf('Average iLQG Optimization Time %f seconds with stddev %f seconds \n', mean(optim_time), std(optim_time))
fprintf('Detected %d Collisions \n', nCollisions)


