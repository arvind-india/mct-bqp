function [cbest, cbest_candidates] = parseTemporal(x, k, cands_percam, num_cams)
    cbest = cell(num_cams,1);
    cbest_candidates = cell(num_cams,1);
    for c = 1:num_cams
        x_temp = x{c}{1};
        [~, best] = max(x_temp);
        best_candidates = cell(length(best),1);
        for bc = 1:length(best)
            best_candidates{bc} = cands_percam{c}{bc}(best(bc),:);
        end
        cbest{c} = best;
        cbest_candidates{c} = best_candidates;
    end
end
