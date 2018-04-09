function plot_output(all_candidates, ground_plane_regions, ground_plane_regions_adjusted, cameras, show_candidates, show_ground_truth, candidates_frame, draw_regions, tracklets, nohomocorrec_tracklets, gnd_detections,overlap_adjusted, num_frames,debug_gnd_truth_frames,start_frames)
    %openfig(floor_image); hold on;
    figure; hold on;

    legends = false;

    % TODO Plot GROUND TRUTH
    colors = {'Orange','Purple','Grey'};
    colors_adjusted = {'Red','Blue','Black'};
    colors_nohomo = {'Salmon','Cyan'};

    for i = 1:length(cameras)
        if show_candidates == true && ~isempty(all_candidates{candidates_frame}) % DEBUG Define a specific frame here for debug to see all candidates of that specific frame
            for j=1:size(all_candidates{candidates_frame}{i},2)
                scatter(all_candidates{candidates_frame}{i}{j}(:,5),all_candidates{candidates_frame}{i}{j}(:,6),14,'d','MarkerFaceColor',rgb('White'),'MarkerEdgeColor',rgb(colors_adjusted{i}))
            end
        end
        if draw_regions == true
            drawPoly(ground_plane_regions{i},colors{i},0.5,false); % Draw regions
            if ~isempty(ground_plane_regions_adjusted{i}) % If we have adjusted them, draw adjustments
                drawPoly(ground_plane_regions_adjusted{i},colors_adjusted{i},0.5,false);
            end
        end
    end
    if draw_regions == true && ~isempty(ground_plane_regions_adjusted{1}) &&  ~isempty(ground_plane_regions_adjusted{2})
        drawPoly(overlap_adjusted, colors_adjusted{3},1.0,false);
    end

    % TODO Plot WITHOUT homography correction
    nohomocorrec_plots = cell(2,1);
    for s = 1:length(nohomocorrec_tracklets)
        camera = nohomocorrec_tracklets{s}(1,1);
        nohomocorrec_plots{camera}{end+1} = nohomocorrec_tracklets{s};
        if s <= length(nohomocorrec_tracklets) - 2 && nohomocorrec_tracklets{s}(1,2) == nohomocorrec_tracklets{s+2}(1,2) + 70
            plot([nohomocorrec_tracklets{s}(2,8) ; nohomocorrec_tracklets{s+2}(2,8)],[nohomocorrec_tracklets{s}(2,9) ; nohomocorrec_tracklets{s+2}(2,9)],'s--','Color',rgb('Gray'));
        end
    end

    % TODO Plot WITH homography correction
    plots = cell(2,1);
    for s = 1:length(tracklets)
        camera = tracklets{s}(1,1);
        plots{camera}{end+1} = tracklets{s};
        if s <= length(tracklets) - 2 && tracklets{s}(1,2) == tracklets{s+2}(1,2) + 70
            plot([tracklets{s}(2,8) ; tracklets{s+2}(2,8)],[tracklets{s}(2,9) ; tracklets{s+2}(2,9)],'k--');
        end
    end

    for i = 1:length(cameras)
        nohomocorrec_plots{i} = cell2mat(transpose(nohomocorrec_plots{i}));
        plots{i} = cell2mat(transpose(plots{i}));
        nohomocorrec_plots{i} = accumarray(nohomocorrec_plots{i}(:,3),(1:size(nohomocorrec_plots{i},1)).',[],@(x){nohomocorrec_plots{i}(x,:)},{});
        plots{i} = accumarray(plots{i}(:,3),(1:size(plots{i},1)).',[],@(x){plots{i}(x,:)},{});
    end

    for i = 1:length(cameras)
        for s = 1:size(nohomocorrec_plots{i},1)
            plot(nohomocorrec_plots{i}{s}(:,8),nohomocorrec_plots{i}{s}(:,9),'o--','Color',rgb(colors_nohomo{i}));
            plot(plots{i}{s}(:,8),plots{i}{s}(:,9),'s-','Color',rgb(colors_adjusted{i}));
        end
        %for matches = 1:size(valid_matchings{1},2)
        %    plot([valid_matchings{1}{matches}(:,8); valid_matchings{2}{matches}(:,8)],[valid_matchings{1}{matches}(:,9); valid_matchings{2}{matches}(:,9)],'k');
        %end
    end

    if show_ground_truth == true
        for f = 1:(num_frames - 1)
            if f == debug_gnd_truth_frames + 1 % DEBUG Always draw one more for debug reasons
                break;
            end
            truth1 = gnd_detections{1}{start_frames(1) + f};
            truth2 = gnd_detections{2}{start_frames(2) + f};
            scatter(truth1(:,8),truth1(:,9),'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange'));
            scatter(truth2(:,8),truth2(:,9),'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple'));
        end
    end
    if legends == true
        if draw_regions == true
            legend('Original cam 40 region', 'Corrected cam 40 region', 'Original cam 19 region', 'Corrected cam 19 region', 'Overlap region', ...
            'Pedestrian 1 -- Cam 40','Corrected Pedestrian 1 -- Cam 40', ...
            'Pedestrian 2 -- Cam 40','Corrected Pedestrian 2 -- Cam 40', ...
            'Pedestrian 1 -- Cam 19','Corrected Pedestrian 1 -- Cam 19',...
            'Pedestrian 2 -- Cam 19','Corrected Pedestrian 2 -- Cam 19',...
            'Initial detections Cam 40','Initial detections Cam 19');
        else
            legend('Ped 1 Cam 40','Ped 1 Cam 40', ...
            'Ped 2 Cam 40','Ped 2 Cam 40', ...
            'Ped 1 Cam 19','Ped 1 Cam 19',...
            'Ped 2 Cam 19','Ped 2 Cam 19',...
            'Initial detections Cam 40','Initial detections Cam 19');
        end
    end
    xlabel('x(m)') % x-axis label
    ylabel('y(m)') % y-axis label
end
