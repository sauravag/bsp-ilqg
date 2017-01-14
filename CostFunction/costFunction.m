function c = costFunction(b, u, goal, L, stDim, stateValidityChecker, varargin)
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
    if isempty(varargin)
        c(i) =  evaluateCost(b(:,i),u(:,i), goal, stDim, L, stateValidityChecker);
    else
        c(i) =  evaluateCost(b(:,i),u(:,i), goal, stDim, L, stateValidityChecker, varargin{1});
    end
end

end

function cost = evaluateCost(b, u, goal, stDim, L, stateValidityChecker, varargin)
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
R_t = 0.2*eye(ctrlDim); % penalize control effort
Q_l = 10*L*eye(stDim); % penalize terminal error
w_cc = 1.0; % penalize collision

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
  
  % if extra arg is 1, get collision cost or if no extra arg, default behaviour
  if ~isempty(varargin)
      if varargin{1} == 1
        nSigma = sigmaToCollide(b,stDim,stateValidityChecker);
    
        cc = -log(chi2cdf(nSigma^2, stDim));
      end
  else
      nSigma = sigmaToCollide(b,stDim,stateValidityChecker);
    
      cc = -log(chi2cdf(nSigma^2, stDim));      
  end
  
end

cost = sc + ic + uc + w_cc*cc;

end