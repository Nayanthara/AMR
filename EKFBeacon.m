function[mu,sigma] = EKFBeacon(mu0,sigma0,Ut,hParams,Zt,R,Qval,skipUpdate)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #4
%   SAJAN, NAYANTHARA

%  Prediction step
mu      = feval('integrateOdom',mu0,Ut);
Gt      = feval('GjacDiffDrive',mu0,Ut);
sigma   = Gt*sigma0*Gt' + R;

if(~skipUpdate)
    [Ht,validZt] = feval('HjacBeacon',mu,hParams,Zt);
    pred         = feval('hBeacon',mu,hParams);
    h            = [];
    Zt_prime     = [];
    for i = 1:size(validZt,1)
        if ~isempty(find(pred(:,1)==validZt(i,1)))
        h = [h; pred(pred(:,1)==validZt(i,1),2:3)];
        Zt_prime = [Zt_prime;validZt(i,2:3)];
        end
    end
    h  = reshape(h,2*size(h,1),1);
    
    if(~isempty(h) && ~isempty(Ht))
       Zt_prime = reshape(Zt_prime,2*size(Zt_prime,1),1);
       Q = Qval*eye(size(Ht,1));
        
        %   Compute Kalman gain
        Kt      = sigma*Ht'*inv(Ht*sigma*Ht'+Q);
        
        %   Update step
        mu      = mu + Kt*(Zt_prime - h);
        sigma   = (eye(size(sigma,1))-Kt*Ht) * sigma;
    end
end
end