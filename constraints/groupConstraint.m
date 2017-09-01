function Cg = groupConstraint(n,k,f,allbbs,allDetections)
    allbbsvec = cell2mat(allbbs');
    T = zeros(n*k);
    %Consider a big group with all the pairwise relationships
    for l1=1:(n*k)
        for l2=1:(n*k)

            %Get i target
            i = ceil(l1/k);
            %Get j target
            j = ceil(l2/k);
            %Compute eij
            bb1 = allDetections{1}{f}(i,:);
            bb2 = allDetections{1}{f}(j,:);
            xx1 = bb1(3) + bb1(5);
            yy1 = bb1(4) + bb1(6);
            xx2 = bb2(3) + bb2(5);
            yy2 = bb2(4) + bb2(6);
            eij = sqrt((xx2-xx1)^2+(yy2-yy1)^2);

            p_i = allbbsvec(l1,1:2);
            p_j = allbbsvec(l2,1:2);
            sp_sigma = (allbbsvec(l2,3)*allbbsvec(l2,4))/2; %Set to half the target size
            T(l1,l2) = exp(-(sqrt((p_i(1)-p_j(1))^2+(p_i(2)-p_j(2))^2) - eij)/(2*sp_sigma^2));
        end
    end
    D = diag(sum(T,2)); %row sum
    Dinvsq = T.^(-1/2);
    Dinvsq(~isfinite(Dinvsq)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
    Dsq = T.^(1/2);
    Tl = Dinvsq*T*Dsq;
    Cg = eye(n*k) - Tl; %Laplacian trick
