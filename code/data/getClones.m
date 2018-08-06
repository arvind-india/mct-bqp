function cands_clones = getClones(N, num_cams, k, targs_clones, overlap)
    cands_clones = cell(num_cams,1);
    % cands_percam = cell(num_cams,1); % Candidates from both cameras for each target
    for c = 1:num_cams
        for t = 1:size(targs_clones{c},1)
            cands_clones{c}{t} = zeros(k,6);
            % Check if inside overlap
            in = polyin(targs_clones{c}(t, 8:9), overlap);
            if in
                for r = 1:size(targs_clones{getOtherCamIdx(c)},1)
                    % Fill the candidates as the targets in the other camera, leave zeros otherwise
                    cands_clones{c}{t}(r,:) = targs_clones{getOtherCamIdx(c)}(r,4:9);
                end
            end
        end
    end
end

function oi = getOtherCamIdx(idx)
  oi = rem(idx,2) + 1; % TODO This is a dirty hack to get 1 if 2 or 2 if 1
end
