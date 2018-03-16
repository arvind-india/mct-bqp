function valid_matchings = getValidMatchings(i, S, score_threshold, n_ov2, n_ov1, targs_in_overlap, assignments, valid_matchings)
    if n_ov2 >= n_ov1
        fprintf('\t\tBest assignment for id %d in cam %d === id %d in cam %d \n', i, targs_in_overlap{1}(i,1),...
        assignments(i), targs_in_overlap{2}(assignments(i),1));
        score = S(i,assignments(i));
    else
        fprintf('\t\tBest assignment for id %d in cam %d === id %d in cam %d \n', assignments(i), targs_in_overlap{1}(assignments(i),1),...
        i, targs_in_overlap{2}(i,1));
        score = S(assignments(i),i);
    end
    fprintf('\t\tScore: %f\n', score);
    % TODO If its a good score, then store merge
    if score < score_threshold
        % TODO Make this work for more than 2 cameras
        if n_ov2 >= n_ov1
            valid_matchings{1}{end+1} = targs_in_overlap{1}(i,:);
            valid_matchings{2}{end+1} = targs_in_overlap{2}(assignments(i),:);
        else
            valid_matchings{1}{end+1} = targs_in_overlap{1}(assignments(i),:);
            valid_matchings{2}{end+1} = targs_in_overlap{2}(i,:);
        end
    end
end
