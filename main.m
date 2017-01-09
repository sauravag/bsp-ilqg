%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Belief Space Planning with iLQG
% Copyright 2017
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
close all;

fprintf('\n A demonstration of the iLQG algorithm for Belief Space Planning \n')

% Set full_DDP=true to compute 2nd order derivatives of the 
% dynamics. This will make iterations more expensive, but 
% final convergence will be much faster (quadratic)
full_DDP = false;

% set up the optimization problem
DYNCST  = @(x,u,i) sys_dyn_cst(x,u,full_DDP);
T       = 500;              % horizon
x0      = [1;1;pi*3/2;0];   % initial state
u0      = .1*randn(2,T);    % initial controls
Op.lims  = [-.5 .5;         % wheel angle limits (radians)
             -2  2];        % acceleration limits (m/s^2)
Op.plot = -1;               % plot the derivatives as well

% prepare the visualization window and graphics callback
figure(9);
set(gcf,'name','car parking','Menu','none','NumberT','off')
set(gca,'xlim',[-4 4],'ylim',[-4 4],'DataAspectRatio',[1 1 1])
grid on
box on

% plot target configuration with light colors
handles = car_plot([0 0 0 0]', [0 0]');
fcolor  = get(handles,'facecolor');
ecolor  = get(handles,'edgecolor');
fcolor  = cellfun(@(x) (x+3)/4,fcolor,'UniformOutput',false);
ecolor  = cellfun(@(x) (x+3)/4,ecolor,'UniformOutput',false);
set(handles, {'facecolor','edgecolor'}, [fcolor ecolor])

% prepare and install trajectory visualization callback
line_handle = line([0 0],[0 0],'color','k','linewidth',2);
plotFn = @(x) set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));
Op.plotFn = plotFn;

% === run the optimization!
[x,u]= iLQG(DYNCST, x0, u0, Op);

% animate the resulting trajectory
figure(9)
handles = [];
for i=1:T
   set(0,'currentfigure',9);
   delete(handles)
   handles = car_plot(x(:,i), u(:,i));
   drawnow    
end
