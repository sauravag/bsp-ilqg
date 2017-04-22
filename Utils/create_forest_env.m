% Create a forest type map with circular obstacles
clc; clear;

fPath = '/Users/sauravagarwal/Desktop/Research/Development/bsp-ilqg/Maps/mapTask1.mat';

load(fPath);

obstacleRadius = 0.3; % radius of 1 obstacles

spacing = 2.5; % distance between obstacle centers

map.obstacleRadius = obstacleRadius;

map.bounds = [0 20 20 0 0;0 0 20 20 0];

map.goal = [17.0;17.75];

map.landmarks = [1 17;18 1]; 

x_oc =  min(map.bounds(1,:))+2*obstacleRadius:spacing: max(map.bounds(1,:))-2*obstacleRadius;
y_oc =  min(map.bounds(2,:))+2*obstacleRadius:spacing: max(map.bounds(2,:))-2*obstacleRadius;

map.obstacles = [];

for x = x_oc
    for y = y_oc
        map.obstacles = [map.obstacles [x;y]];
    end
end

save(fPath,'map');

fprintf('Done \n');