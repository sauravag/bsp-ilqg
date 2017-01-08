%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps
% Copyright 2015
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
        map; % the landmarks that have been mapped
    end
    
    methods (Abstract)
        
        z = getObservation(obj,x, varargin)
                
        H_r = getObservationJacobian(obj,x,v,varargin)
        
        M = getObservationNoiseJacobian(obj,x,v,z)
        
        R = getObservationNoiseCovariance(obj,x,z)
        
        H_m = getObservationFeatureJacobian(obj,x,f);
        
        P_f = getObservationFeatureCovariance(obj,x,z);
        
        innov = computeInnovation(obj,Xprd,Zg,varargin);
        
        feats = getInverseObservation(obj, x, z);
        
        G = getInverseObservationJacobian(obj,x,z);
        
        W = getInverseObservationNoiseJacobian(obj,x,z);
    end
    
    methods
        
        function obj = updateMap(obj, map)
            obj.map = map;
        end
        
        function draw(obj,x,z)
            warning('not implemented');
        end
    end
end