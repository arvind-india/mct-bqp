function [targs_speed, tracks] = update(targs_speed, targs_percam, tracks, best_temporal_candidates, num_cams, dt)

    fprintf('\t 11. Updating motion models \n');
    for c = 1:num_cams
        for ts = 1:size(targs_speed{c},2)
            tx = targs_percam{c}(ts,8);
            ty = targs_percam{c}(ts,9);
            best_cx = best_temporal_candidates{c}{ts}(5);
            best_cy = best_temporal_candidates{c}{ts}(6);
            targs_speed{c}{ts}(1) = (best_cx - tx)/dt;
            targs_speed{c}{ts}(2) = (best_cy - ty)/dt;
        end
    end
    %---------------------------------------------------------------------------
    fprintf('\t 12. Saving results, using results as next targets \n');
    for c = 1:num_cams
        % Increment frame
        targs_percam{c}(:,2) = targs_percam{c}(:,2) + 1;
        for ts = 1:size(targs_percam{c},1)
            targs_percam{c}(ts,4:9) = best_temporal_candidates{c}{ts};
            tracks{c}{end+1} = [targs_percam{c}(ts,4:9) ; best_temporal_candidates{c}{ts}];
        end
    end
end
