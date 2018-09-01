function [targs_speed, tracks] = update(targs_speed, targs_percam, tracks, best_temporal_candidates, num_cams, dt, dataset)

    fprintf('\t 11. Updating motion models \n');
    for c = 1:num_cams
        for ts = 1:size(targs_speed{c},2)
            tx = targs_percam{c}(ts,8);
            ty = targs_percam{c}(ts,9);
            best_cx = best_temporal_candidates{c}{ts}(5);
            best_cy = best_temporal_candidates{c}{ts}(6);
            targs_speed{c}{ts}(1) = (best_cx - tx);
            targs_speed{c}{ts}(2) = (best_cy - ty);
        end
    end
    %---------------------------------------------------------------------------
    fprintf('\t 12. Saving results, using results as next targets \n');
    if strcmp(dataset, 'ucla')
        for c = 1:num_cams
            % Increment frame
            targs_percam{c}(:,7) = targs_percam{c}(:,7) + 1;
            for ts = 1:size(targs_percam{c},1)
                tracks{c}{targs_percam{c}(ts,2) + 1}{end+1} = [[targs_percam{c}(ts,7) - 1 targs_percam{c}(ts,3:6) targs_percam{c}(ts,8:9)] ; targs_percam{c}(ts,7) best_temporal_candidates{c}{ts}];
                targs_percam{c}(ts,3:6) = best_temporal_candidates{c}{ts}(1:4);
                targs_percam{c}(ts,8:9) = best_temporal_candidates{c}{ts}(5:6);
            end
        end
    elseif strcmp(dataset,'hda')
        for c = 1:num_cams
            % Increment frame
            targs_percam{c}(:,2) = targs_percam{c}(:,2) + 1;
            for ts = 1:size(targs_percam{c},1)
                tracks{c}{end+1} = [targs_percam{c}(ts,3:9) ; best_temporal_candidates{c}{ts}];
                targs_percam{c}(ts,3:6) = best_temporal_candidates{c}{ts}(1:4);
            end
        end
    end

end
