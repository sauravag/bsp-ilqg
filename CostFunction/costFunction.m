function c = costFunction(b, u, goal, stDim, collisionChecker)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for vector of states according to cost model given in Section 6 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control
%   stDim: State dimension
%
% Outputs:
%   c: cost estimate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

L = size(b,2);

c = zeros(1,L);

for i=1:L
    c(i) =  stateCost(b(:,i),u(:,i), goal, stDim, L, collisionChecker);
end

end

function c = stateCost(b, u, goal, stDim, L, collisionChecker)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for a states according to cost model given in Section 6 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control
%   stDim: State dimension
%   L: Number of steps in horizon
% Outputs:
%   c: cost estimate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

final = isnan(u(1,:));
u(:,final)  = 0;

ctrlDim = size(u,1);

x = b(1:stDim,1);

sqrtSigma = zeros(stDim, stDim); % principal sqrt of covariance matrix

% Extract columns of principal sqrt of covariance matrix
% right now we are not exploiting symmetry
for d = 1:stDim
    sqrtSigma(:,d) = b(d*stDim+1:(d+1)*stDim, 1);
end

Q_d = 2e-3*eye(stDim); % penalize distance to target
Q_t = eye(stDim);
R_t = 1.1*eye(ctrlDim);
Q_l = L*eye(stDim);

% deviation from goal
delta_x = goal-x;

% final cost
if any(final)
  c = delta_x'*Q_l*delta_x + trace(sqrtSigma*Q_l*sqrtSigma);
else
  c = delta_x'*Q_d*delta_x + u'*R_t*u + trace(sqrtSigma*Q_t*sqrtSigma) + 1e2*collisionChecker(x);
end

end