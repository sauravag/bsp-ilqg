% RRT algorithm in 2D with collision avoidance.
%
% Thanks to Sai Vemprala for original implementation.
%
% nodes:    Contains list of all explored nodes. Each node contains its
%           coordinates, cost to reach and its parent.
%
% Brief description of algorithm:
% 1. Pick a random node q_rand.
% 2. Find the closest node q_near from explored nodes to branch out from, towards
%    q_rand.
% 3. Steer from q_near towards q_rand: interpolate if node is too far away, reach
%    q_new. Check that obstacle is not hit.
% 4. Add q_new to node list.
% 5. Continue until maximum number of nodes is reached or goal is hit.
classdef RRTStar < PlannerBase
    
    properties
        map = []; % state dimension
        stateValidityChecker = [];  % state validity checker
        goalNeighborhood = 0.2; % neighborhood of goal
        motionModel = [];
        EPS = 0.3;
    end
    
    
    methods
        
        function obj = RRTStar(map,mm,svc)
            obj@PlannerBase();
            
            obj.map = map;
            obj.stateValidityChecker = svc;
            obj.motionModel = mm;
            
        end
        
        function [solutionPath,u, figh] = plan(obj,x0,xf) % discrete motion model equation
            
            x_max = obj.map.bounds(1,2);
            y_max = obj.map.bounds(2,3);
            
            numNodes = 5000;
            goalBias = 0.05;
            
            q_start.coord = x0';
            q_start.cost = 0;
            q_start.parent = 0;
            q_goal.coord = xf';
            q_goal.cost = 0;
            
            nodes(1) = q_start;
            
            figh = figure;
            axis([0 x_max 0 y_max])
            drawObstacles(figh,obj.map);
            title('RRT');
            xlabel('X (m)'); ylabel('Y (m)');
            hold on
            
            for i = 1:1:numNodes
                if rand < goalBias
                    q_rand = q_goal.coord;
                else                    
                    q_rand = [rand(1)*x_max rand(1)*y_max];
                end
                plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])
                
                % Break if goal node neighborhood is reached
                for j = 1:1:length(nodes)
                    if norm(nodes(j).coord-q_goal.coord,2) < obj.goalNeighborhood
                        break
                    end
                end
                
                % Pick the closest node from existing list to branch out from
                ndist = [];
                for j = 1:1:length(nodes)
                    n = nodes(j);
                    tmp = norm(n.coord-q_rand,2);
                    ndist = [ndist tmp];
                end
                [val, idx] = min(ndist);
                q_near = nodes(idx);
                
                q_new.coord = obj.steer(q_rand, q_near.coord, val, obj.EPS);
                if obj.isPathValid(q_rand', q_near.coord')
                    line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
                    drawnow
                    hold on
                    q_new.cost = norm(q_new.coord-q_near.coord,2) + q_near.cost;
                    
                    % Within a radius of r, find all existing nodes
                    q_nearest = [];
                    r = 5;
                    neighbor_count = 1;
                    for j = 1:1:length(nodes)
                        if obj.isPathValid(nodes(j).coord',q_new.coord') && norm(nodes(j).coord-q_new.coord,2) <= r
                            q_nearest(neighbor_count).coord = nodes(j).coord;
                            q_nearest(neighbor_count).cost = nodes(j).cost;
                            neighbor_count = neighbor_count+1;
                        end
                    end
                    
                    % Initialize cost to currently known value
                    q_min = q_near;
                    C_min = q_new.cost;
                    
                    % Iterate through all nearest neighbors to find alternate lower
                    % cost paths
                    
                    for k = 1:1:length(q_nearest)
                        if obj.isPathValid(q_nearest(k).coord', q_new.coord') && q_nearest(k).cost + norm(q_nearest(k).coord-q_new.coord,2) < C_min
                            q_min = q_nearest(k);
                            C_min = q_nearest(k).cost + norm(q_nearest(k).coord-q_new.coord,2);
%                             line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');
%                             hold on
                        end
                    end
                    
                    %Update parent to least cost-from node
                    for j = 1:1:length(nodes)
                        if nodes(j).coord == q_min.coord
                            q_new.parent = j;
                        end
                    end
                    
                    % Append to nodes
                    nodes = [nodes q_new];
                end
            end
            
            D = [];
            for j = 1:1:length(nodes)
                tmpdist = norm(nodes(j).coord-q_goal.coord,2);
                D = [D tmpdist];
            end
            
            % Search backwards from goal to start to find the optimal least cost path
            [~, idx] = min(D);
            %q_final = nodes(idx);
            q_goal.parent = idx;
            q_end = q_goal;
            nodes = [nodes q_goal];
            
            % the final trajectory found by RRT
            solutionPath = [q_goal.coord'];
            
            while q_end.parent ~= 0
                start = q_end.parent;
                line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
                hold on
                q_end = nodes(start);
                solutionPath = [solutionPath q_end.coord'];
            end
            
            solutionPath = fliplr(solutionPath);
            u = [];
            
            for i = 1:size(solutionPath,2)-1
                u = [u obj.motionModel.generateOpenLoopControls(solutionPath(:,i),solutionPath(:,i+1))];
            end
                        
        end
        
        function yesno = isPathValid(obj,xt,xs)
            %%%%%%%%%%%%%%%%%%%%%
            % Check path validity
            %%%%%%%%%%%%%%%%%%%%%
            
            steps = ceil(norm((xt-xs),2) / obj.EPS); % number of segments
            
            X = linspace(xs(1),xt(1),steps);
            Y = linspace(xs(2),xt(2),steps);
            
            for i = 1:steps
                
                p = [X(i);Y(i)];
                
                if obj.stateValidityChecker(p) == 0
                    yesno = 0;
                    return;
                end
                
            end
            
            yesno = 1;
        end
        
    end
    
    
end


