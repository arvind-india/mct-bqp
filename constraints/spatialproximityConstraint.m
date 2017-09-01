function Csp = spatialproximityConstraint(n,k,allbbs)
    S = zeros(n*k);
    allbbsvec = cell2mat(allbbs');
    for l1=1:(n*k)
        for l2=1:(n*k)
            p_i = allbbsvec(l1,1:2);
            p_j = allbbsvec(l2,1:2);
            sp_sigma = (allbbsvec(l2,3)*allbbsvec(l2,4))/2; %Set to half the target size
            S(l1,l2) = exp(-((p_i(1)-p_j(1))^2+(p_i(2)-p_j(2))^2)/(2*sp_sigma^2));
        end
    end
    Ds = diag(sum(S,2)); %row sum
    Dsinvsq = Ds.^(-1/2);
    Dsinvsq(~isfinite(Dsinvsq)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
    Dssq = Ds.^(1/2);
    Sl = Dsinvsq*S*Dssq;
    Csp = eye(n*k) - Sl; %Laplacian trick
