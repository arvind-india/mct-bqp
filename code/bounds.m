function b = bounds(k, n, cands_percam, cands_clones, ground_plane_regions, reward_val, num_cams)
    b = cell(num_cams,1);
    % Normal section
    for c = 1:num_cams
        c_b = zeros(k,n);
        b_clones = zeros(k,n);
        for i=1:n
            for j=1:k
                candidate = cands_percam{c}{i}(j,5:6);
                if polyin(candidate,ground_plane_regions{c})
                    c_b(j,i) = reward_val; % TODO Otherwise assign a good weight (note that this is negative!!)
                else
                    % c_b(j,i) = penalize_val; % TODO If the candidate is outside this ground plane region assign it a bad weight
                    c_b(j,i) = -reward_val;
                end
            end
        end
        % Clone sections
        for i=1:n
            for j=1:k
                candidate = cands_clones{c}{i}(j,5:6);
                if candidate(1) ~= 0 && candidate(2) ~= 0 && polyin(candidate,ground_plane_regions{c})
                    % TODO Otherwise assign a good weight
                    b_clones(j,i) = reward_val;
                else
                    % b_clones(j,i) = penalize_val;
                    b_clones(j,i) = -reward_val;
                end
            end
        end

        c_b = c_b(:);
        b_clones = b_clones(:);
        c_b = [c_b ; b_clones];
        b{c} = c_b;
    end
end
