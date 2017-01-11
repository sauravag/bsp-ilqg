function drawLandmarks(h, landmarks)
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw landmarks/beacons in the world.
% Input:
%   h: figure handle
%   landmarks: list of landmark vertices
%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(h)
hold on;

% draw white circle around markers
for l = 1:size(landmarks,2)
    scatter(landmarks(1,l),landmarks(2,l),1000,'filled', 'MarkerFaceAlpha',0.9,'MarkerFaceColor',[1.0 1.0 1.0])
end

plot(landmarks(1,:),landmarks(2,:),'ks',...
    'LineWidth',2,...
    'MarkerSize',8,...
    'MarkerFaceColor',[0.25,0.25,0.25])
end