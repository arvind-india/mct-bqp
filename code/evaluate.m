function correct_tracks = evaluate(f, gnd_detections, tracks, people, start_frames, num_cams, overlap)
    % Show the original tracks
    correct_tracks = cell(num_cams,1);
    for c = 1:num_cams
        correct_tracks{c} = cell(max(people) + 1,1); % + 1 cuz matlab
    end
    colors = {{'Red', 'Aquamarine'} {'Green', 'Blue'}};
    figure;
    hold on;
    for c = 1:num_cams
        dets = cell2mat(gnd_detections{c}(start_frames(c): start_frames(c) + (f-1),:));
        for p = 1:length(people)
            dx = dets(dets(:,2) == people(p),:);
            plot(dx(:,8),dx(:,9),'Color',rgb(colors{c}{p}),'Marker','s');
            for t = 1:(size(dx,1)-1)
                correct_tracks{c}{people(p) + 1}{end+1} = [[dx(t,7) dx(t,3:6) dx(t,8:9)] ; [dx(t+1,7) dx(t+1,3:6) dx(t+1,8:9)]];
            end
        end

    end
    colors = {{'Orange', 'Turquoise'} {'Lime', 'Navy'}};
    for c = 1:num_cams
        for p = 1:length(people)
            tr = tracks{c}{people(p)+1};
            for frame = 1:size(tr,1)
                gr = tr{frame};
                plot(gr(:,6),gr(:,7),'Color',rgb(colors{c}{p}),'Marker','X');
            end
        end
    end
    % Compute error
    cae_error = [0.0 0.0];
    for c = 1:num_cams
        for ppl = 1:size(correct_tracks{c},1)
            if isempty(correct_tracks{c}{ppl}) == 0
                for fr = 1:size(correct_tracks{c}{ppl},1)
                    correct = correct_tracks{c}{ppl}{fr};
                    pred = tracks{c}{ppl}{fr};
                    cae_error = cae_error + sum(abs(correct(:,6:7) - pred(:,6:7)));
                end
            end
        end
    end
    cae_error = sum(cae_error);


    drawPoly(overlap, 'Black', 0.5, false);
    xlabel('x(m)'); ylabel('y(m)');
    legend('Cam 1 - GT 1','Cam 1 - GT 2','Cam 2 - GT 1','Cam 2 - GT 2', 'Cam 1 - TR 1', 'Cam 1 - TR 2','Cam 2 -- TR 1','Cam 2 -- TR 2', 'Overlap Region')
    title(['Ground-truth and tracks on the ground plane. Cumulative Average Error: ' num2str(cae_error)]);

end
