function Cg = groupConstraint_v2(n,k,targs)
    T = zeros(n*k);
    sp_sigma = 0.1;
    t = cell2mat(targs');
    %Consider a big group with all the pairwise relationships
    for l1 = 1:(n*k)
        for l2 = 1:(n*k)
            i = ceil(l1/k); %Get i target
            j = ceil(l2/k); %Get j target

            %Compute eij
            xx1 = t(i,9); yy1 = t(i,10);
            cam1 = t(i,1); cam2 = t(j,1);
            xx2 = t(j,9); yy2 = t(j,10);
            eij = abs(xx2 - xx1) + abs(yy2 - yy1);
            if cam1 ~= cam2
              T(l1,l2) = 1.2 * exp(-(sqrt((xx2-xx1)^2+(yy2-yy1)^2) - eij)/(2*sp_sigma^2));
            end
            if i ~= j
              T(l1,l2) = exp(-(sqrt((xx2-xx1)^2+(yy2-yy1)^2) - eij)/(2*sp_sigma^2));
            else
              T(l1,l2) = 100000.0;
            end
        end
    end
    D = diag(sum(T,2)); %row sum
    Dinvsq = T.^(-1/2);
    Dinvsq(~isfinite(Dinvsq)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
    Dsq = T.^(1/2);
    Tl = Dinvsq*T*Dsq;
    Cg = eye(n*k) - Tl; %Laplacian trick
end
