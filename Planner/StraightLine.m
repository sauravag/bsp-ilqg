classdef StraightLine < PlannerBase
     
    properties
        map = []; % state dimension
        stateValidityChecker = [];  % state validity checker
        goalNeighborhood = []; % neighborhood of goal
        motionModel = [];
    end
    
    
    methods
        
        function obj = StraightLine(~,mm,~)
        
            obj@PlannerBase();                       
            obj.motionModel = mm;
            
        end
        
        function [x,u,figh] = plan(obj, x0, xf) % Generates a
            
            figh = [];
            u = obj.motionModel.generateOpenLoopControls(x0,xf);
            x = x0;
            
            for i = 1:size(u,2)
                x = [x obj.motionModel.evolve(x(:,end),u(:,i),obj.motionModel.zeroNoise)];
            end
            
        end
        
    end
end