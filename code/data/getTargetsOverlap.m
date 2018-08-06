function [targs_in_overlap, n_o, N_o] = getTargetsOverlap(targs, overlap, cameras)
    targs_in_overlap = {};
    for t = 1:size(targs,1)
        %if targs(t,1) == 1
        %    plot(targs(t,8),targs(t,9),'r+');
        %end
        %if targs(t,1) == 2
        %    plot(targs(t,8),targs(t,9),'b+');
        %end
        if polyin([targs(t,8) targs(t,9)],overlap)
            targs_in_overlap{end+1} = targs(t,:);
        end
    end
    if ~isempty(targs_in_overlap)
        targs_in_overlap = cell2mat(targs_in_overlap');
        targs_in_overlap = (accumarray(targs_in_overlap(:,1),(1:size(targs_in_overlap,1)).',[],@(x){targs_in_overlap(x,:)},{}));
        n_o = cell(length(cameras),1);
        for i = 1:length(cameras)
            n_o{i} = size(targs_in_overlap{i},1);
        end

        N_o = sum(cell2mat(n_o));
    end
end
