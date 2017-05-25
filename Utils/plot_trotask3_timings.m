% Plot Graphs for Task 3, moving start away from Goal
% We use forest environment
% List of Start Points: 
% ID    x	y
%  1   1.725	1.725
%  2   3.975	3.975
%  3   6.225	6.225
%  4   8.475	8.475
%  5   10.725	10.725
%  6   12.975	12.975
%  7   15.225	15.225
%  8   17.475	17.475

%% Setup
baseDirectory = '/home/saurav/MATLAB/TRO/';

nStarts = 8;

start = [1.725 3.975 6.225 8.475 10.725 12.975 15.225 17.475;...
            1.725 3.975 6.225 8.475 10.725 12.975 15.225 17.475];
        
goal = [17.0000;...
        17.7500];
    
s2gDistance = zeros(1,nStarts);


for i = 1:nStarts
    
    s2gDistance(i) = norm(start(:,i) - goal);
     
end
    
s2gDistance = fliplr(s2gDistance);

s2gDistance = [s2gDistance 142 268.7006]; % the last distance is for a really long experiment

%% ILQG DATA
ilqg_avgOptimTime = [2.7699   13.5303   26.6656   35.2866  101.3331  190.2307  199.6180  176.0970, 2541, 12698.11];
ilqg_stdDevOptimTime = [0.3178    2.5620    4.9643    7.6897   64.3799  101.2705   50.5458   43.7460 0.0 0.0];

mfh = figure;
% errorbar(s2gDistance,ilqg_avgOptimTime,ilqg_stdDevOptimTime)
plot(s2gDistance,ilqg_avgOptimTime)
xlabel('\textbf{Distance (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
ylabel('\textbf{Optimization Time (s)}','fontsize',14,'fontweight','bold','Interpreter','latex');
set(mfh,'Units','Inches');
pos = get(mfh,'Position');
set(mfh,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(strcat(baseDirectory,'ilqg_task3_timing'),'-depsc');

%% Rollout Data
rollout_avgAddTime = [0.03, 0.024, 0.1044, 0.0662, 0.059, 0.0558, 0.1176, 0.1272, 0.11, 0.087];
% rollout_stdDevAddTime = [];

rollout_avgDPSolveTime = [0.1096, 0.1194, 0.1096,0.1162, 0.1132, 0.1184, 0.1108, 0.1096, 1.147, 1.18];
% rollout_stdDevDPSolveTime = [];


mfh2 = figure;
% errorbar(s2gDistance,rollout_avgAddTime,rollout_stdDevAddTime)
plot(s2gDistance,rollout_avgAddTime);
xlabel('\textbf{Distance (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
ylabel('\textbf{Time to Add Start State to Graph (s)}','fontsize',14,'fontweight','bold','Interpreter','latex');
% xticks([0 10 20 30 40 50 60 200 300])
ylim([0 0.2])
set(mfh2,'Units','Inches');
pos = get(mfh2,'Position');
set(mfh2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(strcat(baseDirectory,'rollout_task3_addstartstate'),'-depsc');

mfh3 = figure;
% errorbar(s2gDistance,rollout_avgDPSolveTime,rollout_stdDevDPSolveTime)
plot(s2gDistance,rollout_avgDPSolveTime);
xlabel('\textbf{Distance (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
ylabel('\textbf{Time to Solve Dynamic Programming (s)}','fontsize',14,'fontweight','bold','Interpreter','latex');
ylim([0 2])
% xticks([0 10 20 30 40 50 60 200 300])
set(mfh3,'Units','Inches');
pos = get(mfh3,'Position');
set(mfh3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(strcat(baseDirectory,'rollout_task3_dpsolve'),'-depsc');

