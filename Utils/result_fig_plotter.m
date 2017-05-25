% Plot an iLQG solution and save as EPS file

baseDirectory = '/home/saurav/MATLAB/TRO/';
fname = 'ilqg_rollout_task3_solution';

mfh = gcf;
plot(rx,ry,'.g','LineWidth',2);
plot(cx(:,1:50:2564),cy(:,1:50:2564),'.g')
xlabel('\textbf{X (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
ylabel('\textbf{Y (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
set(mfh,'Units','Inches');
pos = get(mfh,'Position');
set(mfh,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(strcat(baseDirectory,fname),'-depsc');