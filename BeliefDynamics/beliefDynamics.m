function b_next = beliefDynamics(b, u, motionModel, obsModel)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Propagate beliefs according to approach given in Section 4.1 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control input
%   motionModel: Robot motion model
%   obsModel: Observation model
%
% Outputs:
%   b_next: Updated belief vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

horizon = size(b,2);

b_next = zeros(size(b));

for i=1:horizon    
    b_next(:,i) = updateSingleBelief(b(:,i), u(:,i), motionModel, obsModel);
end


end

function b_next = updateSingleBelief(b, u, motionModel, obsModel)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Propagate single belief according to approach given in Section 4.1 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control input
%   motionModel: Robot motion model
%   obsModel: Observation model
%
% Outputs:
%   b_next: Updated belief vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isnan(u(1,:))
    u = zeros(size(u));
end

% get the state space dimension
stDim = motionModel.stDim;

% Extract robot state
x = b(1:stDim,1);

P = zeros(stDim, stDim); % covariance matrix

% Extract columns of principal sqrt of covariance matrix
% right now we are not exploiting symmetry
for d = 1:stDim
    P(:,d) = b(d*stDim+1:(d+1)*stDim, 1);
end

% update state
processNoise = motionModel.zeroNoise; % 0 process noise
x_next = motionModel.evolve(x,u,processNoise); 

% Get motion model jacobians
A = motionModel.getStateTransitionJacobian(x,u,processNoise);
G = motionModel.getProcessNoiseJacobian(x,u,processNoise);
Q = motionModel.getProcessNoiseCovariance(x,u);

% Get observation model jacobians
z = obsModel.getObservation(x_next, 'nonoise');
obsNoise = zeros(size(z));
H = obsModel.getObservationJacobian(x,obsNoise);
M = obsModel.getObservationNoiseJacobian(x,obsNoise,z);
R = obsModel.getObservationNoiseCovariance(x,z);

% update P 
P_prd = A*P*A' + G*Q*G';
S = H*P_prd*H' + M*R*M';
K = (P_prd*H')/S;
P_next = (eye(stDim) - K*H)*P_prd;

% update belief
% W = zeros(stDim+stDim^2,2);
% W(1:stDim,:) = sqrtm(K*H*T);
% 
% w = mvnrnd(zeros(1,stDim), eye(stDim),1);
% w = w';

g_b_u = zeros(size(b));
g_b_u(1:stDim,1) = x_next;

g_b_u(stDim+1:end,1) = P_next(:);

% for d = 1:stDim
%     g_b_u(d*stDim+1:(d+1)*stDim,1) = P_next(:,d);
% end

b_next = g_b_u ;%+ W*w;

end
% 
% function b_next = updateSingleBelief(b, u, motionModel, obsModel)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Propagate single belief according to approach given in Section 4.1 
% % of Van Den Berg et al. IJRR 2012
% %
% % Input:
% %   b: Current belief vector
% %   u: Control input
% %   motionModel: Robot motion model
% %   obsModel: Observation model
% %
% % Outputs:
% %   b_next: Updated belief vector
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% if isnan(u(1,:))
%     u = zeros(size(u));
% end
% 
% % get the state space dimension
% stDim = motionModel.stDim;
% 
% % Extract robot state
% x = b(1:stDim,1);
% 
% sqrtSigma = zeros(stDim, stDim); % principal sqrt of covariance matrix
% 
% % Extract columns of principal sqrt of covariance matrix
% % right now we are not exploiting symmetry
% for d = 1:stDim
%     sqrtSigma(:,d) = b(d*stDim+1:(d+1)*stDim, 1);
% end
% 
% % update state
% processNoise = motionModel.zeroNoise; % 0 process noise
% x_next = motionModel.evolve(x,u,processNoise); 
% 
% % Get motion model jacobians
% A = motionModel.getStateTransitionJacobian(x,u,processNoise);
% G = motionModel.getProcessNoiseJacobian(x,u,processNoise);
% 
% % Get observation model jacobians
% z = obsModel.getObservation(x_next, 'nonoise');
% obsNoise = zeros(size(z));
% H = obsModel.getObservationJacobian(x,obsNoise);
% M = obsModel.getObservationNoiseJacobian(x,obsNoise,z);
% 
% % update sqrtSigma 
% T = A*sqrtSigma*(A*sqrtSigma)' + G*G';
% K = (T*H')/(H*T*H' + M*M');
% sqrtSigma_next = sqrtm(T - K*H*T);
% 
% % update belief
% % W = zeros(stDim+stDim^2,2);
% % W(1:stDim,:) = sqrtm(K*H*T);
% % 
% % w = mvnrnd(zeros(1,stDim), eye(stDim),1);
% % w = w';
% 
% g_b_u = zeros(size(b));
% g_b_u(1:stDim,1) = x_next;
% 
% for d = 1:stDim
%     g_b_u(d*stDim+1:(d+1)*stDim,1) = sqrtSigma_next(:,d);
% end
% 
% b_next = g_b_u ;%+ W*w;
% 
% end