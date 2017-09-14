% Plot an iLQG solution and save as EPS file
uiopen('/Users/sauravagarwal/Google Drive/2017_SLAP_Rollout_FIRM/DATA-Sims/September2017/iLQG Single Homotopy/run1/iLQG-solution.fig',1)

if isunix ==1
    [~,username] = system('whoami');
    baseDirectory = ['/home/',username(1:end-1),'/MATLAB/TRO/SingleHomotopy/'];
    % Mac is unix so have to check here
    if ismac==1
        baseDirectory = ['/Users/',username(1:end-1),'/Documents/MATLAB/TRO/'];
    end
end

% path = load('/Users/sauravagarwal/Google Drive/2017_SLAP_Rollout_FIRM/DATA-Sims/September2017/Rollout Forest Single Homotopy/run2/RobotPath.csv');
% cov = load('/Users/sauravagarwal/Google Drive/2017_SLAP_Rollout_FIRM/DATA-Sims/September2017/Rollout Forest Single Homotopy/run3/RobotCovarianceEllipse.csv');
% 
% rx = path(:,1);
% ry = path(:,2);
% 
% cx = cov(:,1);
% cy = cov(:,2);
% 
% [rows,cols] = size(cx);
% 
% cx = reshape(cx,40,rows/40);
% cy = reshape(cy,40,rows/40);

fname = 'ilqg_rollout_task_singlehomotopy_solution';

mfh = gcf;
% plot(rx,ry,'.g','LineWidth',2);
% plot(cx(:,1:50:2564),cy(:,1:50:2564),'.g')
xlabel('\textbf{X (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
ylabel('\textbf{Y (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
set(mfh,'Units','Inches');
pos = get(mfh,'Position');
set(mfh,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
mfh.InvertHardcopy = 'off';
print(strcat(baseDirectory,fname),'-depsc');
saveas(gcf,strcat(baseDirectory,fname),'png')


