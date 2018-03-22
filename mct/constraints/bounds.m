function c_b = bounds(cam,k,n,cands_percam,ground_plane_regions,penalize_val,reward_val)
    c_b = zeros(k,n);
    for i=1:n
        for j=1:k
            candidate = cands_percam{cam}{i}(j,5:6);
            if polyin(candidate,ground_plane_regions{cam})
                % TODO If the candidate is outside this ground plane region assign it a bad weight
                c_b(j,i) = reward_val;
            else
                % TODO Otherwise assign a good weight
                c_b(j,i) = penalize_val;
            end
        end
    end
    c_b = c_b(:);
end
