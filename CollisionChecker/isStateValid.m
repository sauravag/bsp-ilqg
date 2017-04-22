function yesno = isStateValid(x, map, dynamicObs)
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

N = 50; % discretize robot body
delta_theta = 2*pi/N;
theta = 0:delta_theta:2*pi-delta_theta;

% robot perimeter
ptPerimiter = repmat(x,1,N) + R*[cos(theta);sin(theta)] ;

% % check if robot is within boundary
bounds_xv = map.bounds(1,:);
bounds_yv = map.bounds(2,:);
inbounds = inpolygon(ptPerimiter(1,:),ptPerimiter(2,:),bounds_xv,bounds_yv);

% if robot not within bounds return false
if sum(inbounds) ~= size(ptPerimiter,2)
    yesno = 0;
    return;
end

dx = map.obstacles(1,:) - x(1);
dy = map.obstacles(2,:) - x(2);

c2c = sqrt(dx.^2 + dy.^2);                

if any(c2c <= R + map.obstacleRadius)
    yesno = 0;
    return;
end

if dynamicObs == 1
    dx = map.dynamicObstacles(1,:) - x(1);
    dy = map.dynamicObstacles(2,:) - x(2);
    
    c2c = sqrt(dx.^2 + dy.^2);
    
    if any(c2c <= R + map.obstacleRadius)
        yesno = 0;
        return;
    end
end

yesno = 1;

end

% function yesno = isStateValid(x, map, dynamicObs)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Check if robot is in collision with obstacles
% % Input:
% %   x: robot state
% %   map: obstacle map
% %   varargin: robot radius to override default
% %
% % Output:
% %   yesno: 1 if robot is not in collision (valid state)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% % robot radius
% global ROBOT_RADIUS;
% R = ROBOT_RADIUS;
%
% N = 50; % discretize robot body
% delta_theta = 2*pi/N;
% theta = 0:delta_theta:2*pi-delta_theta;
%
% % robot perimeter
% ptPerimiter = repmat(x,1,N) + R*[cos(theta);sin(theta)] ;
% robot = ptPerimiter;
%
% % get points on lines joining center to perimeter point
% for i =1:N
%     lpts_x = linspace(x(1),ptPerimiter(1,i),N);
%     lpts_y = linspace(x(2),ptPerimiter(2,i),N);
%     robot = [robot [lpts_x;lpts_y]];
% end
%
%
% % % check if robot is within boundary
% bounds_xv = map.bounds(1,:);
% bounds_yv = map.bounds(2,:);
% inbounds = inpolygon(ptPerimiter(1,:),ptPerimiter(2,:),bounds_xv,bounds_yv);
%
% % if robot not within bounds return false
% if sum(inbounds) ~= size(ptPerimiter,2)
%     yesno = 0;
%     return;
% end
%
% for i=1:length(map.obstacles)
%     obs = map.obstacles{i};
%     %     ts = tic;
%     collided = inpolygon(robot(1,:),robot(2,:),obs(1,:),obs(2,:));
%     %     fprintf('Time to CC: %f s \n', toc(ts))
%     % if robot perimiter point inside polygon it collided
%     if any(collided)
%         yesno = 0;
%         return;
%     end
%
% end
%
% if dynamicObs == 1
%     for i=1:length(map.dynamicObs)
%         obs = map.dynamicObs{i};
%         %     ts = tic;
%         collided = inpolygon(robot(1,:),robot(2,:),obs(1,:),obs(2,:));
%         %     fprintf('Time to CC: %f s \n', toc(ts))
%         % if robot perimiter point inside polygon it collided
%         if any(collided)
%             yesno = 0;
%             return;
%         end
%
%     end
% end
%
% yesno = 1;
%
% end