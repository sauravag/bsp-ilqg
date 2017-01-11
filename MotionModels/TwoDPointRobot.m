%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Belief Space Planning with iLQG
% Copyright 2017
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 2D Point robot with additive process noise.
classdef TwoDPointRobot < MotionModelBase
    properties (Constant = true) % note that all properties must be constant, because we have a lot of copies of this object and it can take a lot of memory otherwise.
        stDim = 2; % state dimension
        ctDim = 2;  % control vector dimension
        wDim = 2;   % Process noise (W) dimension
        P_Wg = diag([0.01,0.01].^2); % covariance of state-additive-noise
        sigma_b_u = [0.0;0.0]; % A constant bias intensity (std dev) of the control noise
        eta_u = [0;0]; % A coefficient, which makes the control noise intensity proportional to the control signal       
        zeroNoise = [0;0];
    end
    
    methods
        
        function obj = TwoDPointRobot(dt)
            obj@MotionModelBase();      
            obj.dt = dt;
        end
        
        function x_next = evolve(obj,x,u,w) % discrete motion model equation            
            x_next = x + u*obj.dt + w;
        end
        
        function A = getStateTransitionJacobian(obj,x,u,w) % state Jacobian
            A = eye(2);
        end
        
        function B = getControlJacobian(obj,x,u,w) % control Jacobian
            B = obj.dt*eye(2);
        end
        
        function G = getProcessNoiseJacobian(obj,x,u,w) % noise Jacobian
            G = eye(2);
        end
        
        function Q = getProcessNoiseCovariance(obj,x,u) % compute the covariance of process noise based on the current poistion and controls
            Q = obj.P_Wg;
        end
        
        function w = generateProcessNoise(obj,x,u) % simulate (generate) process noise based on the current state and controls
            w = mvnrnd(zeros(obj.stDim,1),obj.P_Wg)';
        end
        
    end
end