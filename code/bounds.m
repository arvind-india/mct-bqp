function c_b = bounds(k, n, cands_percam, cands_clones, ground_plane_regions, penalize_val, reward_val, num_cams)
    c_b = cell(num_cams,1);
    % Normal section
    for c = 1:num_cams
        b = zeros(k,n);
        b_clones = zeros(k,n);
        for i=1:n
            for j=1:k
                candidate = cands_percam{c}{i}(j,5:6);
                if polyin(candidate,ground_plane_regions{c})
                    b(j,i) = reward_val; % TODO Otherwise assign a good weight (note that this is negative!!)
                else
                    b(j,i) = penalize_val; % TODO If the candidate is outside this ground plane region assign it a bad weight
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
                    b_clones(j,i) = penalize_val;
                end
            end
        end

        b = b(:);
        b_clones = b_clones(:);
        b = [b ; b_clones];
        c_b{c} = b;
    end
    c_b = cell2mat(c_b);
end
