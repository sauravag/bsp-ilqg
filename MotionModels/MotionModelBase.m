%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Belief Space Planning with iLQG
% Copyright 2017
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef MotionModelBase < handle
    properties (Abstract, Constant) % note that all properties must be constant, because we have a lot of copies of this object and it can take a lot of memory otherwise.
        stDim; % state dimension
        ctDim;  % control vector dimension
        wDim;   % Process noise (W) dimension
        P_Wg; % covariance of state-additive-noise
        sigma_b_u; % A constant bias intensity (covariance) of the control noise
        eta_u; % A coefficient, which makes the control noise intensity proportional to the control signal       
        zeroNoise;
        ctrlLim; % control limits
    end
    
    properties
        dt = 0.0; % delta_t for time discretization
    end
    
    methods (Abstract)
        x_next = evolve(x,u,w) % discrete motion model equation
        
        A = getStateTransitionJacobian(x,u,w) % state Jacobian
        
        B = getControlJacobian(x,u,w) % control Jacobian
        
        G = getProcessNoiseJacobian(x,u,w) % noise Jacobian
        
        Q = getProcessNoiseCovariance(x,u) % compute the covariance of process noise based on the current poistion and controls
        
        w = generateProcessNoise(x,u) % simulate (generate) process noise based on the current state and controls
        
        U = generateOpenLoopControls(obj,x0,xf) % generate open loop controls from start to end point
        
    end
end