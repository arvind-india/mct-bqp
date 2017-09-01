function [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(A,b, Aeq, Beq, labels,method)

N = size(A,1);

% initialization:
% x_0 should be in the active set, don't mess this up
x_0     = zeros(N,1);
[~,ib]  = unique(labels, 'rows','last');
x_0(ib) = 1;
S_0     = x_0;
alpha_0 = 1;

opts.maxIter  = 5000;  % max number of iteration, I found out the opts.TOL is more important in the convergence in my problem
%opts.DG       = 1e-2;  % This set the duality gap and you can change it if you want to change the tolerance for convergence
opts.DG       = 1e-4;
opts.eps      = 1e-12; % eps in matlab = 1e-16

t1 = tic;
[minx,minf,x_t,f_t,~] = FW_crowd_swap(x_0, S_0, alpha_0, A, b,Aeq, Beq, opts);
t1_end = toc(t1);
