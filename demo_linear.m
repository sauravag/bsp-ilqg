%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2015, Yuval Tassa
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
% 
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in
%       the documentation and/or other materials provided with the distribution
%     * Neither the name of the University of Washington nor the names
%       of its contributors may be used to endorse or promote products derived
%       from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function demo_linear
% A demo of iLQG/DDP with a control-limited LTI system.
clc;

fprintf(['A demonstration of the iLQG/DDP algorithm\n'...
'with a random control-limited time-invariant linear system.\n'...
'for details see\nTassa, Mansard & Todorov, ICRA 2014\n'...
'\"Control-Limited Differential Dynamic Programming\"\n'])

% make stable linear dynamics
h = .01;        % time step
n = 10;         % state dimension
m = 2;          % control dimension
A = randn(n,n);
A = A-A';       % skew-symmetric = pure imaginary eigenvalues
A = expm(h*A);  % discrete time
B = h*randn(n,m);

% quadratic costs
Q = h*eye(n);
R = .1*h*eye(m);

% control limits
Op.lims = ones(m,1)*[-1 1]*.6;

% optimization problem
DYNCST  = @(x,u,i) lin_dyn_cst(x,u,A,B,Q,R);
T       = 1000;             % horizon
x0      = randn(n,1);       % initial state
u0      = .1*randn(m,T);    % initial controls

% run the optimization
iLQG(DYNCST, x0, u0, Op);


function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = lin_dyn_cst(x,u,A,B,Q,R)

% for a positive-definite quadratic, no control cost (indicated by the 
% iLQG function using nans), is equivalent to u=0
u(isnan(u)) = 0;

if nargout == 2
    f = A*x + B*u;
    c = 0.5*sum(x.*(Q*x),1) + 0.5*sum(u.*(R*u),1);
else 
    N   = size(x,2);
    fx  = repmat(A, [1 1 N]);
    fu  = repmat(B, [1 1 N]);
    cx  = Q*x;
    cu  = R*u;
    cxx = repmat(Q, [1 1 N]);
    cxu = repmat(zeros(size(B)), [1 1 N]);
    cuu = repmat(R, [1 1 N]);
    [f,c,fxx,fxu,fuu] = deal([]);
end