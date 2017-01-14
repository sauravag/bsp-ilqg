function nSigma = sigmaToCollide(b,stDim,stateValidityChecker)
%%%%%%%%%%%%%%%%%%%%%%%
% Compute std devs to collision based on 
% Section 5 of Van den Berg et al. IJRR 2012
%
% Inputs:
%   b: belief state
%   stateValidityChecker: function for checking collision
%
% Output:
%   nSigma: number of std devs robot should deviate to collide
%%%%%%%%%%%%%%%%%%%%%%

nSigma = 3.5*ones(1,size(b,2));

global ROBOT_RADIUS
R_orig =  ROBOT_RADIUS; % save robot radius
    
for i = 1:size(b,2)
    
    x = b(1:stDim,i);

    P = zeros(stDim, stDim); % covariance matrix

    % Extract columns of principal sqrt of covariance matrix
    % right now we are not exploiting symmetry
    for d = 1:stDim
        P(:,d) = b(d*stDim+1:(d+1)*stDim, i);
    end

    eigval = eig(P); % get eigen values

    lambda = max(eigval); % get largest eigen val

    d = sqrt(lambda); % distance along 1 std dev  

    % number of standard deviations at which robot collides
    % at s = 0, f goes to infinite so not good -> better to use small value of 0.01
    for s = 0.01:0.2:3.21

        % inflate robot radius 
        ROBOT_RADIUS = R_orig + s*d;

        % if robot collided
        if stateValidityChecker(x) == 0    

            nSigma(i) = s;            
            
            break;
        end
    end

    
end

ROBOT_RADIUS = R_orig; % reset robot radius
end