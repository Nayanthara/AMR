function[mu,sigma] = EKF(mu0,sigma0,gFunc,gJacFunc,Ut,hFunc,hJacFunc,hParams,Zt,validIndices,R,Q,tolerance)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #4
%   SAJAN, NAYANTHARA
    
%   Resize matrices
    Q = Q(validIndices,validIndices);

%  Prediction step
    mu      = feval(gFunc,mu0,Ut);
    Gt      = feval(gJacFunc,mu0,Ut);
    sigma   = Gt*sigma0*Gt' + R;
    Ht      = feval(hJacFunc,mu,hParams);
    Ht      = Ht(validIndices,:);
    h       = feval(hFunc,mu,hParams);
    h       = h(validIndices,:);
    
%   Compute Kalman gain
    Kt      = sigma*Ht'*inv(Ht*sigma*Ht'+Q);
    
%   Update step
    Zt      = Zt(validIndices,:);
    error   = Zt - h; 
    if std(error) < tolerance
        mu      = mu + Kt*error;
        sigma   = (eye(size(sigma,1))-Kt*Ht) * sigma;
    end
end