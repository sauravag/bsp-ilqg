function [g,c,gb,gu,gbb,gbu,guu,cb,cu,cbb,cbu,cuu] = beliefDynCost(b,u,xf,L,full_DDP,motionModel,obsModel, collisionChecker)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A utility function that combines belief dynamics and cost
% uses helper function finite_difference() to compute derivatives
% Inputs:
%   b: belief
%   u: controls
%   xf: target state
%   L: Total segments
%   full_DDP: whether to use 2nd order derivates of dynamics
%   motionModel: robot's motion model
%   obsModel: Sensing model
%   collisionChecker: collision checking with obstacles
%
% Outputs:
%   g: belief update using belief dynamics
%   c: cost 
%   gx: belief dynamics derivate w.r.t belief state
%   gu: belief dynamics derivate w.r.t control
%   gbb: 2-nd order derivate of belief dynamics derivate w.r.t belief state
%   gbu: belief dynamics derivate w.r.t belief state and control
%   guu: 2-nd order derivate of belief dynamics derivate w.r.t control
%   cb: cost func derivate w.r.t belief state
%   cu: cost func derivate w.r.t control
%   cbb: 2-nd order derivate of cost func derivate w.r.t belief state
%   cbu: cost func derivate w.r.t belief state and control
%   cuu: 2-nd order derivate of cost func derivate w.r.t control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

beliefDim = size(b,1);
ctDim = motionModel.ctDim;

if nargout == 2
    g = beliefDynamics(b, u, motionModel, obsModel);
    c = costFunction(b, u, xf, L, motionModel.stDim, collisionChecker);
else
    % belief state and control indices
    ib = 1:beliefDim;
    iu = beliefDim+1:beliefDim+ctDim;
    
    % dynamics first derivatives
    xu_dyn  = @(xu) beliefDynamics(xu(ib,:),xu(iu,:),motionModel, obsModel);
    J       = finiteDifference(xu_dyn, [b; u]);
    gb      = J(:,ib,:);
    gu      = J(:,iu,:);

    % dynamics second derivatives
    if full_DDP
        xu_Jcst = @(xu) finite_difference(xu_dyn, xu);
        JJ      = finiteDifference(xu_Jcst, [b; u]);
        JJ      = reshape(JJ, [4 6 size(J)]);
        JJ      = 0.5*(JJ + permute(JJ,[1 3 2 4])); %symmetrize
        gbb     = JJ(:,ib,ib,:);
        gbu     = JJ(:,ib,iu,:);
        guu     = JJ(:,iu,iu,:);
    else
        [gbb,gbu,guu] = deal([]);
    end
    
    % cost first derivatives
    xu_cost = @(xu) costFunction(xu(ib,:),xu(iu,:),xf,L,motionModel.stDim, collisionChecker);
    J       = squeeze(finiteDifference(xu_cost, [b; u]));
    cb      = J(ib,:);
    cu      = J(iu,:);
    
    % cost second derivatives
    tStart = tic;
    xu_Jcst = @(xu) squeeze(finiteDifference(xu_cost, xu));
    JJ      = finiteDifference(xu_Jcst, [b; u]);
    JJ      = 0.5*(JJ + permute(JJ,[2 1 3])); %symmetrize
    cbb     = JJ(ib,ib,:);
    cbu     = JJ(ib,iu,:);
    cuu     = JJ(iu,iu,:);        
    fprintf('Time to do Cost Hessian: %f seconds\n', toc(tStart))

    [g,c] = deal([]);
end
end