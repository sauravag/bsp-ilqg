% Create a forest type map with circular obstacles
clc; clear;

fPath = '/home/saurav/Research/Development/bsp-ilqg/Maps/mapTask3.mat';

load(fPath);

obstacleRadius = 0.5; % radius of 1 obstacles

spacing = 5.0; % distance between obstacle centers

map.obstacleRadius = obstacleRadius;

map.bounds = [0 200 200 0 0;0 0 200 200 0];

map.goal = [103.5;103.5];

map.start = [3.5;3.5];

distance = norm(map.goal-map.start)

map.landmarks = [1:10:190;1:10:190]; 

x_oc =  min(map.bounds(1,:))+2*obstacleRadius:spacing: max(map.bounds(1,:))-2*obstacleRadius;
y_oc =  min(map.bounds(2,:))+2*obstacleRadius:spacing: max(map.bounds(2,:))-2*obstacleRadius;

map.obstacles = [];

for x = x_oc
    for y = y_oc
        map.obstacles = [map.obstacles [x;y]];
    end
end

sPath = '/home/saurav/Research/Development/bsp-ilqg/Maps/mapTask3_long_2.mat';

save(sPath,'map');

fprintf('Done \n');