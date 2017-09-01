function [minx,minf,x_t,f_t,S_k] = FW_crowd_swap(x_0, S_0, alpha_0, A, b,Aeq, Beq, opts) 
% x_0 : variable initialization 
% S_0 : active set initialization 
% alpha_0 : weight initialization 
% A : the matrix in the cost function of the QP
% b : is the vector of the linear cost functions 


max_step    = 1;
it           = 1;
minf = 0;
minx = [];
% init:

x_t         = x_0;
S_k         = S_0;
alpha_k     = alpha_0;
cost_fun =@(y) .5 * y' *A * y + b' * y;

% optimization: 
ctype=repmat('B',1,length(x_0));
options = cplexoptimset();
options.Display = 'off';
gap = 100;

while it <= opts.maxIter
  it = it + 1; 

  % gradient:
  grad        = A * x_t + b;

  % cost function:
  f_t = cost_fun(x_t);

  % Find Descent direction 
  y_T      = cplexlp(grad,[],[],Aeq,Beq,zeros(size(Aeq,2),1),ones(size(Aeq,2),1));
  d_T      = y_T - x_t;
  
  % Find acsent direction 
  if ~isempty(S_k) 
    id_S   = away_step(grad, S_k);
    y_S    = S_k(:, id_S);
    alpha_max = alpha_k(id_S);
  end 
  
  d_S = y_T-y_S;
  step_T = -  (grad' * d_T) / ( d_T' * A * d_T );
  step_S = -  (grad' * d_S) / ( d_S' * A * d_S );
 

  if it==2 || cost_fun(y_T) < minf
      minx = y_T;
      minf = cost_fun(y_T);
  end


  if gap < opts.DG
    fprintf('end of FW: reach small duality gap (gap=%f)\n', gap);
    break
  end
  
  % Pick the best improvement
  delta_T = cost_fun(x_t)-cost_fun(x_t+step_T*(d_T));
  delta_S = cost_fun(x_t)-cost_fun(x_t+step_S*(d_S));
  
  % choose direction (between normal and away):
  if isempty(S_k) || delta_T>delta_S
    is_aw = false;
    y = y_T;
    d = d_T; 
    max_step = 1;
    step = max(0, min(step_T, max_step )); 
  else
    is_aw = true;
    y = y_S;
    d = d_S;
    max_step = alpha_max;
    step = max(0, min(step_S, max_step )); 
  end
  
  % duality gap:
  gap = - d' * grad;
  
  if step < opts.DG
    fprintf('end of FW: step is too small (step=%f)\n', step)
    break
  end

  [S_k, alpha_k] = update_active_set(S_k, alpha_k, y, step, max_step, is_aw);

  x_t = x_t + step * d; 
 
end

y   = cplexmilp(-x_t,[],[],Aeq,Beq,[],[],[],[],[],ctype,[],options);
fy  = cost_fun(y);

if fy < minf
  minf = fy;
  minx = y;
end

function id = away_step(grad, S)
    % find the ascend direction(yk?argmax<y,?f(xk)>)
    s = S' * grad;
    [~,id] = max(s);
    id = id(1);
end


end
