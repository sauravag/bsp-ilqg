% Plot an iLQG solution and save as EPS file
uiopen('iLQG-solution.fig',1)

if isunix ==1
    [~,username] = system('whoami');
    baseDirectory = ['/home/',username(1:end-1),'/MATLAB/'];
    % Mac is unix so have to check here
    if ismac==1
        baseDirectory = ['/Users/',username(1:end-1),'/Documents/MATLAB/'];
    end
end

path = load('RobotPath.csv');
cov = load('RobotCovarianceEllipse.csv');

rx = path(:,1);
ry = path(:,2);

cx = cov(:,1);
cy = cov(:,2);

[rows,cols] = size(cx);

cx = reshape(cx,40,rows/40);
cy = reshape(cy,40,rows/40);

fname = 'ilqg_rollout_task3_solution';

mfh = gcf;
plot(rx,ry,'g','LineWidth',2);
for i = 1:50:size(cx,2)
    plot([cx(:,i);cx(1,i)],[cy(:,i);cy(1,i)],'g', 'LineWidth',2)
end
xlabel('\textbf{X (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
ylabel('\textbf{Y (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
set(mfh,'Units','Inches');
pos = get(mfh,'Position');
set(mfh,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
mfh.InvertHardcopy = 'off';
print(strcat(baseDirectory,fname),'-depsc');
saveas(gcf,strcat(baseDirectory,fname),'png')


