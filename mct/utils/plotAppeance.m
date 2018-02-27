function plotAppeance(c_a,i,n,k,cameraListImages,f,targs_percam,cameras,cands_percam,start_frames)
    % DEBUG
    figure; hold on;
    subplot(2,2,1), imshow(cameraListImages{i}{start_frames(i)+f});
    drawBBs(targs_percam{i}(:,4:7), 'Yellow', 'hda');
    title(['Targets (BB) in camera ' cameras{i}]);
    subplot(2,2,2), imshow(cameraListImages{i}{start_frames(i)+f+1});
    title(['Candidates (BB) in camera ' cameras{i}]);
    o = cell2mat(transpose(cands_percam{i}));
    drawCandidateBBs(o, 'Green', 'hda', k);
    subplot(2,2,3), imshow(cameraListImages{i}{start_frames(i)+f});
    title(['Targets (+) in camera ' cameras{i}]);
    for t=1:size(targs_percam{i},1) % NOTE Targets are represented by their feet
        hold on;
        plot(targs_percam{i}(t,4)+targs_percam{i}(t,6)/2,targs_percam{i}(t,5)+targs_percam{i}(t,7) - 10,'y+');
    end
    subplot(2,2,4), imshow(cameraListImages{i}{start_frames(i)+f+1});
    title(['Candidates (*) in camera ' cameras{i}]);
    for t=1:size(cands_percam{i},2) % NOTE Candidates are represented by their feet
        for c=1:size(cands_percam{i}{t},1)
            hold on;
            plot(cands_percam{i}{t}(c,1)+cands_percam{i}{t}(c,3)/2,cands_percam{i}{t}(c,2)+cands_percam{i}{t}(c,4) - 10,'g*');
        end
    end

    % Get chunks of k from c_a
    %chunks = reshape(c_a,k,n);
    %for li=1:n
        % What is the location of the target in the discretized grid
    %    gr = -reshape(chunks(:,li),5,5);
    %    figure;
    %    image(gr);
    %end
end
