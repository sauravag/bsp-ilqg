function c = costFunction(b,u, stDim, L)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for vector of states according to cost model given in Section 6 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control
%   stDim: State dimension
%   L: Time horizon
% Outputs:
%   c: cost estimate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T = size(b,2);

c = zeros(1,T);

for i=1:T
    c(i) =  stateCost(b(:,i),u(:,i), stDim, L);
end

end

function c = stateCost(b,u, stDim, L)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for a states according to cost model given in Section 6 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control
%   stDim: State dimension
%   L: Time horizon
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

Q_t = eye(stDim);
R_t = eye(ctrlDim);
Q_l = 10*L*eye(stDim);

% final cost
if any(final)
  c = x'*Q_l*x + trace(sqrtSigma*Q_l*sqrtSigma);
else
  c = u'*R_t*u + trace(sqrtSigma*Q_t*sqrtSigma);
end

end