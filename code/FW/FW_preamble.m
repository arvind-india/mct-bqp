function [H,F,Aeq,Beq,labels] = FW_preamble(N,k,a,m,G,b,Alpha,Zeta,n1,n2)
    % Puts the optimization variables in a format that the FW optimization can handle
    % Inputs
    %  n: the homographies for each camera
    %  k: candidates per target
    %  c_a: appearance cues
    %  c_m: motion cues
    %  Cg:  group constraints matrix

    % Output
    %  Variables for the FW optimization

    setTrackerParams;
    % Prepare the inputs to the Frank Wolfe
    Aeq = zeros(N,k*N);
    labels = zeros(k*N,1);
    idx = 1;
    for i=1:N
        Aeq(i,(i-1)*k+1:i*k) = ones(k,1);
        labels(idx:(idx+k-1)) = ones(k,1)*i;
        idx = idx + k;
    end
    Beq = ones(N,1);

    % Model the problem
    H = sparse(G);
    alpha_vec = [ones(n1*k,1) * Alpha(1); ones(n2*k,1) * Alpha(2)];
    zeta_vec = [ones(n1*k,1) * Zeta(1); ones(n2*k,1) * Zeta(2)];
    F = alpha_vec .* a + zeta_vec .* m + b;
    %F = Zeta(1) * m + b;
end
