%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Belief Space Planning with iLQG
% Copyright 2017
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
close all;

fprintf('\n A demonstration of the iLQG algorithm for Belief Space Planning \n')

dt = 0.1;
mm = TwoDPointRobot(dt);
om = TwoDRangeModel([1,2],[1, 2;1,2]);
x = [0;0];
Sigma = [1 0.01;0.01, 1];
sqrtSigma = sqrtm(Sigma);

b = [x;sqrtSigma(:)];
u = [1;0];

b = repmat(b,1,100);
u = randn(2,100);

% for i=1:100
b = beliefDynamics(b, u, mm, om);
% end

c = costFunction(b, u, 2, 100);