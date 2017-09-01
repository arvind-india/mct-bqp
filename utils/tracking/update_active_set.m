function [S_k, alpha_k] = update_active_set(S_k, alpha_k, v_k, gamma_k, gamma_max, is_aw)

% Active set update the weights alpha associated with the solution found at
% every iterate. Depending on weather an swap step is chosen or a fw step
% the update is different. For a good explanation please refer to the
% follwing paper: "On the Global Linear Convergence of Frank-Wolfe
% Optimization Variants" by Julien and Jaggi, NIPS 2015

S_0 = S_k;
a_0 = alpha_k;

if ~is_aw % normal step:
    if isempty(S_k) || gamma_k == gamma_max
        S_k = v_k;
        alpha_k = 1;
    else
        [is_in, id] =ismember(S_k', v_k', 'rows');
        is_in = sum(is_in)~=0;
        id = id == 1;
        
        alpha_k = alpha_k * ( 1 - gamma_k);
        
        if is_in
            alpha_k(id) = alpha_k(id) + gamma_k;
        else
            S_k = [ S_k , v_k];
            alpha_k = [alpha_k , gamma_k];
        end
    end
    
else % away step
    
    [~, id] =ismember(S_k', v_k', 'rows');
    id = id ==1;
    
    alpha_k = alpha_k * ( 1 + gamma_k);
    
    if gamma_k == gamma_max % drop step
        S_k(:,id) = [];
        alpha_k(id)=[];
    else
        alpha_k(id) = alpha_k(id) - gamma_k;
    end
    
end

end