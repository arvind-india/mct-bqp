function valid_matchings = getValidMatchings_fw(n_o,cameras,opt_results_percam_o,targs_in_overlap,valid_matchings)
    possible_matchings = eye(n_o{1},n_o{2});

    for c = 1:length(cameras)
        for i = 1:size(opt_results_percam_o{c},1)
            for j = 1:size(opt_results_percam_o{c},2)
                if opt_results_percam_o{c}(j,i) == 1
                    o_c = rem(c,2) + 1; % TODO This is a dirty hack
                    %Targ i from camera c is associated with targ j from camera o_c
                    fprintf(['\t Camera ' num2str(c) ': ' num2str(targs_in_overlap{c}(i,3)) ' <==> Camera ' num2str(o_c) ': ' num2str(targs_in_overlap{o_c}(j,3)) '\n']);
                    % TODO Weight by the actual vectors we previously generated
                    possible_matchings(i,j) = possible_matchings(i,j) + 1;
                end
            end
        end
    end
    % TODO Weight by the actual vectors we previously generated
    for m = 1:size(possible_matchings,1)
        possible_matchings(m,:) = possible_matchings(m,:)./sum(possible_matchings(m,:));
    end
    % TODO From the possible matchings pick best matching, in the future make it probabilistic
    for m = 1:size(possible_matchings,1)
        [~, idx] = max(possible_matchings(m,:));

        valid_matchings{1}{end+1} = targs_in_overlap{1}(m,:);
        valid_matchings{2}{end+1} = targs_in_overlap{2}(idx,:);

    end
end
