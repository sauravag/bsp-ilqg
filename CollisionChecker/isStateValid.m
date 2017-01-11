function yesno = isStateValid(x, map)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check if robot is in collision with obstacles
% Input:
%   x: robot state
%   map: obstacle map
%   varargin: robot radius to override default
%  
% Output:
%   yesno: 1 if robot is not in collision
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N = 100; % discretize robot body perimeter

% robot radius
global ROBOT_RADIUS;
R = ROBOT_RADIUS;

theta = linspace(0,2*pi,N);

% robot body
robot = repmat(x,1,N) + R*[cos(theta);sin(theta)];

% check if robot is within boundary
bounds_xv = map.bounds(1,:);
bounds_yv = map.bounds(2,:);

inbounds = inpolygon(robot(1,:),robot(2,:),bounds_xv,bounds_yv);

% if robot not within bounds return false
if sum(inbounds) ~= N
    yesno = 0;
    return;
end

for i=1:length(map.obstacles)
    obs = map.obstacles{i};
    collided = inpolygon(robot(1,:),robot(2,:),obs(1,:),obs(2,:));
    
    % if robot perimiter point inside polygon it collided
    if any(collided)
        yesno = 0;
        return;
    end
        
end

yesno = 1;

end