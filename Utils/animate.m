
function animate(figh, plotFn, b0, b_nom, u_nom, L, motionModel, obsModel)

stDim = motionModel.stDim;

xt = b0(1:stDim); % true state of robot
x = b0(1:stDim); % estimated mean
P = zeros(stDim); % covariance

% unpack covariance from belief vector
for d = 1:stDim
    P(:,d) = b0(d*stDim+1:(d+1)*stDim, 1);
end

rh = []; % robot disk drawing handle

figure(figh);
plot(b_nom(1,:),b_nom(2,:),'g', 'LineWidth',2);

for i = 1:size(u_nom,2)
    
    b = [x(:);P(:)]; % current belief
                  
    u = u_nom(:,i) + L(:,:,i)*(b - b_nom(:,i));
    
    % update robot 
    processNoise = motionModel.generateProcessNoise(xt,u); % process noise
    xt = motionModel.evolve(xt,u,processNoise); 
   
    % Get motion model jacobians and predict pose
    zeroProcessNoise = motionModel.generateProcessNoise(xt,u); % process noise     
    x_prd = motionModel.evolve(x,u,processNoise); % predict robot pose
    A = motionModel.getStateTransitionJacobian(x,u,zeroProcessNoise);
    G = motionModel.getProcessNoiseJacobian(x,u,zeroProcessNoise);
    Q = motionModel.getProcessNoiseCovariance(x,u);
    
    % Get observation model jacobians
    z = obsModel.getObservation(xt); % true observation  
    z_prd = obsModel.getObservation(x_prd,'nonoise'); % predicted observation
    zerObsNoise = zeros(size(z));
    H = obsModel.getObservationJacobian(x,zerObsNoise);
    M = obsModel.getObservationNoiseJacobian(x,zerObsNoise,z);
    R = obsModel.getObservationNoiseCovariance(x,z);
    
    % update P
    P_prd = A*P*A' + G*Q*G';
    S = H*P_prd*H' + M*R*M';
    K = (P_prd*H')/S;
    P = (eye(stDim) - K*H)*P_prd;    
    x = x_prd + K*(z - z_prd);
    
    b = [x(:);P(:)];
    
    drawResult(plotFn,b,2);
        
    delete(rh)
    rh = scatter(x(1),x(2),100,'filled','MarkerFaceAlpha',1,'MarkerFaceColor',[1.0 0.0 0.0]);
    drawnow;
    pause(0.005);
end

end