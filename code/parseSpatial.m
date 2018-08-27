function [cbest, cbest_candidates, spatial_matchings] = parseSpatial(x, k, cands_clones, num_cams)
    cbest = cell(num_cams,1);
    cbest_candidates = cell(num_cams,1);
    for c = 1:num_cams
        x_spat = x{c}{2};
        [~, best] = max(x_spat);
        best_candidates = cell(length(best),1);
        for bc = 1:length(best)
            best_candidates{bc} = cands_clones{c}{bc}(best(bc),:);
        end
        cbest{c} = best;
        cbest_candidates{c} = best_candidates;
    end
    % Matchings
    spatial_matchings = cell(num_cams,1);
    cbest_aux = cell2mat(cbest);
    % TODO If number of targets is not the same a warning will probably trigger here
    for sm = 1:size(cbest_aux,2)
        if range(cbest_aux(:,sm)) == 0 % NOTE Example: This means targ 1 from cam 1 picked targ 1 from cam 2 and targ 1 from cam 2 picked targ 1 from cam 1

            % TODO Maybe another criteria (maximum voting)
            spatial_matchings{sm} = [cbest_candidates{1}{sm} ; cbest_candidates{2}{sm}];

        end
    end
end
