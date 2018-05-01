function [metrics,tracks,gnd_tracks] = evaluate(ground_plane_regions,ground_plane_regions_adjusted,gnd_detections,tracklets,start_frames)
    % ground truth: gnd_detections
    % tracklets: tracks

    % Merge tracks
    tracks = cell(2,1);
    for i = 1:length(tracklets)
        for k = 1:size(tracklets{i},1)
            if tracklets{i}(1,1) == 1
                tracks{1}{end+1} = [tracklets{i}(k,1) tracklets{i}(k,8:9)];
            else
                tracks{2}{end+1} = [tracklets{i}(k,1) tracklets{i}(k,8:9)];
            end
        end
    end
    tracks{1} = cell2mat(transpose(tracks{1}));
    tracks{2} = cell2mat(transpose(tracks{2}));
    gnd_tracks = cell(2,1); metrics = cell(2,1);
    for id = 1:2
        for j = 1:2
            frame = start_frames(id) + j;
            for k = 1:size(gnd_detections{id}{frame},1)
                gnd_tracks{id}{end+1} = [gnd_detections{id}{frame}(k,1) gnd_detections{id}{frame}(k,8:9)];
            end
        end
        gnd_tracks{id} = cell2mat(transpose(gnd_tracks{id}));
    end
    for id = 1:2
        metrics{id} = abs(tracks{id}(:,2:3) - gnd_tracks{id}(:,2:3));
    end
end
