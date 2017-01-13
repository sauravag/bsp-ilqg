function yesno = isStateValid(x, map)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check if robot is in collision with obstacles
% Input:
%   x: robot state
%   map: obstacle map
%   varargin: robot radius to override default
%  
% Output:
%   yesno: 1 if robot is not in collision (valid state)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% robot radius
global ROBOT_RADIUS;
R = ROBOT_RADIUS;

N = 30; % discretize robot body
delta_theta = 2*pi/N;
theta = 0:delta_theta:2*pi-delta_theta;

% robot perimeter
ptPerimiter = repmat(x,1,N) + R*[cos(theta);sin(theta)] ;
robot = ptPerimiter;

% get points on lines joining center to perimeter point
for i =1:N
    lpts_x = linspace(x(1),ptPerimiter(1,i),N/2);
    lpts_y = linspace(x(2),ptPerimiter(2,i),N/2);    
    robot = [robot [lpts_x;lpts_y]];
end
    
    
% % check if robot is within boundary
% bounds_xv = map.bounds(1,:);
% bounds_yv = map.bounds(2,:);
% inbounds = inpolygon(robot(1,:),robot(2,:),bounds_xv,bounds_yv);
% 
% % if robot not within bounds return false
% if sum(inbounds) ~= size(robot,2)
%     yesno = 0;
%     return;
% end

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