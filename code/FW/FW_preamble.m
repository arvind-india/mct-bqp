function [H,F,Aeq,Beq,labels] = FW_preamble(N,k,a,m,G,b,Alpha,Zeta,n1,n2)
    % Variables for the FW optimization
    a = cell2mat(a);
    m = cell2mat(m);
    b = cell2mat(b);
    % Prepare the inputs to the Frank Wolfe
    Aeq = zeros(N,2*k*N);
    labels = zeros(2*k*N,1);
    idx = 1;
    for i=1:N
        Aeq(i,(i-1)*2*k+1:i*2*k) = ones(2*k,1);
        labels(idx:(idx+2*k-1)) = ones(2*k,1)*i;
        idx = idx + 2*k;
    end
    Beq = ones(N,1);

    % Model the problem
    H = sparse(G);
    alpha_vec = [ones(n1*k,1) * Alpha(1); ones(n2*k,1) * Alpha(2); ones(n1*k,1); ones(n2*k,1)];
    zeta_vec = [ones(n1*k,1) * Zeta(1); ones(n2*k,1) * Zeta(2); ones(n1*k,1); ones(n2*k,1)];
    F = alpha_vec .* a + zeta_vec .* m + b;
    %F = Zeta(1) * m + b;
end
