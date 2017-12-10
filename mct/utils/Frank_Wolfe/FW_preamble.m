function [A,b,Aeq,Beq,labels] = FW_preamble(n,k,c_a,c_m,c_nm,Cg)
    % Puts the optimization variables in a format that the FW optimization can handle
    % Inputs
    %  n: the homographies for each camera
    %  k: candidates per target
    %  c_a: appearance cues
    %  c_m: motion cues
    %  c_nm: neighbourhood cues
    %  Csp: matrix for spatial proximity
    %  Cg:  group constraints matrix

    % Output
    %  Variables for the FW optimization

    setTrackerParams;
    % Prepare the inputs to the Frank Wolfe
    Aeq = zeros(n,k*n);
    for i=1:n
        Aeq(i,(i-1)*k+1:i*k) = ones(k,1);
    end
    Beq = ones(n,1);

    labels = zeros(k*n,1);
    idx = 1;
    for t=1:n
        labels(idx:(idx+k-1)) = ones(k,1)*t;
        idx = idx + k;
    end

    % Model the problem
    A = sparse(Cg);
    b = c_a + zeta*c_m + eta*c_nm;
