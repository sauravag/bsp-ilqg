function drawObstacles(h,obstacles)
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw obstacles in the world. 
% Input:
% h: figure handle
% obstacles: list of obstacle vertices
%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(h)
hold on;

for i = 1:length(obstacles)
    obs = obstacles{i};
    fill(obs(1,:),obs(2,:),'m');
end

end