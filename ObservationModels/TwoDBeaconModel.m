%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Belief Space Planning with iLQG
% Copyright 2017
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Measure signal to beacons, signal strength falls quadratically with distance.
% Robot can see all beacons
classdef TwoDBeaconModel < ObservationModelBase
    
    properties(Constant = true)
        obsDim = 1;
        obsNoiseDim = 1;
    end
    
    properties
        sigma_b = 0.005; 
    end
    
    methods
        
        function obj = TwoDBeaconModel(landmarkIDs, landmarkPoses)
            obj@ObservationModelBase();
            obj.landmarkIDs = landmarkIDs;
            obj.landmarkPoses = landmarkPoses;                                    
        end
        
        function z = getObservation(obj, x, varargin)
            % getObservation, Get what sensor sees conditioned on varargin.            
            % getObservation(obj, x) all visible features with noise
            % getObservation(obj, x, 'nonoise') all visible features with no noise measurements
            % Output :
            % z Observation vector
            % idfs Ids of observations
            
            % if there are no landmarks to see
            if isempty(obj.landmarkPoses) == 1               
                error('There are no landmarks to see');                                
            end
            
            range = obj.computeRange(x);                        
                                                          
            if nargin == 2 % noisy observations
                v = obj.computeObservationNoise(range);
                z = 1./(range.^2+1) + v;
                
            elseif nargin > 2 && strcmp('nonoise',varargin{1}) == 1 % nonoise
                z = 1./(range.^2+1);
                
            else                
                error('unknown inputs')                
            end
            
        end
        
        function H = getObservationJacobian(obj,x, v)
            % Compute observation model jacobian w.r.t state
            % Inputs:
            % x: robot state
            % f: feature pose
            % Outputs:
            % H: Jacobian matrix
            
            dx = x(1) - obj.landmarkPoses(1,:);
            dy = x(2) - obj.landmarkPoses(2,:);
            
            r = sqrt(dx.^2 + dy.^2);                         
            
            H = zeros(size(r,2),2);
            
            for i = 1:size(r,2)
                    H(i,:) = (-2/(r(i)^2+1)^2)*[dx(i) dy(i)];
            end            
            
        end
        
        function M = getObservationNoiseJacobian(obj,x,v,z)
            n = length(z);
            M = eye(n);
        end
        
        function R = getObservationNoiseCovariance(obj,x,z)
                        
            noise_std = repmat(obj.sigma_b,size(z,1),1);
            
            R = diag(noise_std.^2);
            
        end                                                      
        
        function v = computeObservationNoise(obj,z)
            
            noise_std = repmat(obj.sigma_b,size(z,1),1);
            
            v = randn(size(z,1),1).*noise_std;
        end
        
        function innov = computeInnovation(obj,Xprd,Zg)
                        
            z_prd = obj.getObservation(Xprd, 'nonoise');
            
            innov = Zg - z_prd;
        end
        
        function range = computeRange(obj, x)
            
            % Compute exact observation
            dx= obj.landmarkPoses(1,:) - x(1);
            dy= obj.landmarkPoses(2,:) - x(2);
            
            range = sqrt(dx.^2 + dy.^2);                
            
            range = range';
         end
        
    end
    
end
