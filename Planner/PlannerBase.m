classdef PlannerBase < handle
    properties (Abstract)
        map; % state dimension
        stateValidityChecker;  % state validity checker
        goalNeighborhood; % neighborhood of goal
    end
    
    
    methods (Abstract)
        [x,u,figh] = plan(x0,xf) % Generates a
    end
    
    methods
        function A = steer(obj, qr, qn, val, eps)
            qnew = [0 0];
            
            % Steer towards qn with maximum step size of eps
            if val >= eps
                qnew(1) = qn(1) + ((qr(1)-qn(1))*eps)/norm(qr-qn,2);
                qnew(2) = qn(2) + ((qr(2)-qn(2))*eps)/norm(qr-qn,2);
            else
                qnew(1) = qr(1);
                qnew(2) = qr(2);
            end
            A = [qnew(1), qnew(2)];
        end
               
    end
end