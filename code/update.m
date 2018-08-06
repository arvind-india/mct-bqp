fprintf('\t 12.Updating motion models...\n');
for i=1:N
    for j=1:k
        if optimization_results(j,i) == 1
            motion_models{i}(1:2) = cands{i}(j,5:6); % Update Position
            motion_models{i}(3:4) = (cands{i}(j,5:6) - targs(i,8:9))/dt; % Update Speed dt is 1/fps
        end
    end
end
%---------------------------------------------------------------------------
fprintf('\t 13.Saving results, using results as next targets...\n');
for i=1:N
    for j=1:k
        if optimization_results(j,i) == 1
            tracklets{end+1} = [targs(i,:); targs(i,1:3) cands{i}(j,:)];
            targs(i,4:9) = cands{i}(j,:);
        end
    end
end
end
