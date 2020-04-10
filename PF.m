function[particleSet] = PF(particleSet0,gFunc,hFunc,hParams,Ut,Zt,map)

setSize = size(particleSet0,1);
particleSet =[];
weights = [];
validIndices = find(~isnan(Zt(:,1))& Zt(:,1)> 0.2);
Zt = Zt(validIndices,:);
if ~isempty(Zt)
    for m=1:setSize
        particle = mvnrnd(feval(gFunc,particleSet0(m,:)',Ut)',0.001*eye(3));
        particleSet = [particleSet; particle];
        if checkFeasibility(particle(1:2),map)==0
            weight = 0;
        else
            h = feval(hFunc,particle',hParams);
            h = h(validIndices,:);
            Q = 0.01*eye(9);
            weight = mvnpdf(Zt',h',Q(validIndices,validIndices));
        end
        weights = [weights; weight];
    end
    weights = weights./sum(weights);
    indices = randsample(1:setSize,setSize,true,weights');
    particleSet = particleSet(indices,:);
    weights = weights(indices);
    [weights,indices]=sort(weights,'descend');
    particleSet = particleSet(indices,:);
else
    particleSet=particleSet0;
end

end
