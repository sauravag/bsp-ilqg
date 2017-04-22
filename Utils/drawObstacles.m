function drawObstacles(h,map)
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw obstacles in the world. 
% Input:
% h: figure handle
% obstacles: list of obstacle vertices
%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(h)
hold on;

R = map.obstacleRadius;

for i = 1:size(map.obstacles,2)
    obs = map.obstacles(:,i);
    lp = obs - [R;R];
    pos = [lp' 2*R 2*R];
    rectangle('Position',pos','Curvature',[1 1],'FaceColor','m');
end

end