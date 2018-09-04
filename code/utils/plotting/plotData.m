function plotData(targs_percam, cands_percam, best_temporal_candidates, best_spatial_candidates, next_images, images, num_cams, dataset, show_candidates, ground_plane_regions)
    figure
    % For each camera
    colors = {'Lime', 'Salmon', 'Aquamarine', 'Orange'};
    k = 1;
    for c = 1:num_cams
        subplot(3,2,k);
        % Draw targets
        imshow(images{c});
        drawBBs(targs_percam{c}(:,3:6), rgb(colors{k}), 2);
        title(['Cam ' num2str(c) ', frame t'])
        k = k+1;
        subplot(3,2,k);
        imshow(next_images{c});
        % Draw candidates
        for cand = 1:size(cands_percam{c},2)
            drawBBs(cands_percam{c}{cand}(:,1:4), rgb(colors{k}), 1);
        end
        best_cands = cell2mat(best_temporal_candidates{c});
        drawBBs(best_cands(:,1:4), rgb('Blue'), 1);
        title(['Cam ' num2str(c) ', frame t+1 (tracking on cam plane)'])
        k = k+1;
    end

    % For each camera
    subplot(3,2,[k k+1]);
    k = 1;
    hold on;
    for c = 1:num_cams
        % Draw targets in the ground plane
        scatter(targs_percam{c}(:,8), targs_percam{c}(:,9), 'MarkerEdgeColor', rgb(colors{k}), 'MarkerFaceColor', rgb(colors{k}), 'LineWidth', 0.6)
        for t = 1:size(targs_percam{c},1)
            if best_spatial_candidates{c}{t}(5) ~= 0 && best_spatial_candidates{c}{t}(6) ~= 0
                plot([targs_percam{c}(t,8) ; best_spatial_candidates{c}{t}(5)], [targs_percam{c}(t,9) ; best_spatial_candidates{c}{t}(6)],':');
            end
        end
        drawPoly(ground_plane_regions{c}, colors{k}, 0.5, false);
        k = k+1;
        % Draw candidates in the ground plane
        if show_candidates == true
            for cand = 1:size(cands_percam{c},2)
                scatter(cands_percam{c}{cand}(:,5), cands_percam{c}{cand}(:,6),'+', 'MarkerEdgeColor', rgb(colors{k}), 'MarkerFaceColor', rgb(colors{k}), 'LineWidth', 0.3);
            end
        end
        k = k+1;
    end
    title('Ground Plane');
    xlabel('x(m)')
    ylabel('y(m)')
end