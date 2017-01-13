function c = costFunction(b, u, goal, L, stDim, stateValidityChecker)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for vector of states according to cost model given in Section 6 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control
%   goal: target state
%   L: Total segments
%   stDim: state space dimension for robot
%   stateValidityChecker: checks if state is in collision or not
% Outputs:
%   c: cost estimate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

c = zeros(1,size(b,2));

for i=1:size(b,2)
    c(i) =  evaluateCost(b(:,i),u(:,i), goal, stDim, L, stateValidityChecker);
end

end

function cost = evaluateCost(b, u, goal, stDim, L, stateValidityChecker)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for a states according to cost model given in Section 6 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control
%   goal: target state
%   stDim: State dimension
%   L: Number of steps in horizon
%   stateValidityChecker: checks if state is in collision or not
% Outputs:
%   c: cost estimate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

final = isnan(u(1,:));
u(:,final)  = 0;

ctrlDim = size(u,1);

x = b(1:stDim,1);

P = zeros(stDim, stDim); % covariance matrix

% Extract columns of principal sqrt of covariance matrix
% right now we are not exploiting symmetry
for d = 1:stDim
    P(:,d) = b(d*stDim+1:(d+1)*stDim, 1);
end

Q_t = 10*eye(stDim); % penalize uncertainty
R_t = 0.25*eye(ctrlDim); % penalize control effort
Q_l = 10*L*eye(stDim); % penalize terminal error
w_cc = 1;

% deviation from goal
delta_x = goal-x;

% collision cost
cc = 0;

% State Cost
sc = 0;

% information cost
ic = 0;

% control cost
uc = 0;

% final cost
if any(final)
    
  sc = delta_x'*Q_l*delta_x;
  
  ic = trace(P*Q_l*P);
  
else
      
  ic = trace(P*Q_t*P);
  
  uc = u'*R_t*u;
  
  cc = w_cc*collisionCost(x,P,stateValidityChecker); % takes about 0.0035 s
  
end

cost = sc + ic + uc + cc;

end

function f = collisionCost(x,P,stateValidityChecker)
%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for collision based on 
% Section 5 of Van den Berg et al. IJRR 2012
%
% Inputs:
%   x: robot state (belief mean)
%   P: Covariance matrix
%   stateValidityChecker: function for checking collision
%
% Output:
%   f: collision cost
%%%%%%%%%%%%%%%%%%%%%%

stDim = size(x,1);

eigval = eig(P); % get eigen values
 
lambda = max(eigval); % get largest eigen val

d = sqrt(lambda); % distance along 1 std dev

global ROBOT_RADIUS
R_orig =  ROBOT_RADIUS; % save robot radius

% number of standard deviations at which robot collides
% at s = 0, f goes to infinite so not good -> better to use small value of 0.01
for s = [0.01 1 2 3 4] 
    % inflate robot radius 
    ROBOT_RADIUS = R_orig + s*d;
    
    % if robot collided
    if stateValidityChecker(x) == 0    
        
        f = -log(chi2cdf(s^2, stDim));                
        
        ROBOT_RADIUS = R_orig; % reset robot radius
        
        return;
    end
end

f = 0;
ROBOT_RADIUS = R_orig; % reset robot radius
end













