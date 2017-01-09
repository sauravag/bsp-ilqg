%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Belief Space Planning with iLQG
% Copyright 2017
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef ObservationModelBase < handle
    properties (Abstract, Constant) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        obsDim; % dimension of the observation vector
        obsNoiseDim; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
    end
    
    properties
        landmarkIDs; % All landmark IDS in the world
        landmarkPoses; % landmark locations
    end
    
    methods (Abstract)
        
        z = getObservation(obj,x, varargin)
                
        H = getObservationJacobian(obj, x)
        
        M = getObservationNoiseJacobian(obj,x,v,z)
        
        R = getObservationNoiseCovariance(obj,x,z)

        v = computeObservationNoise(obj,z)
        
        innov = computeInnovation(obj,Xprd,Zg)
    end
    
end