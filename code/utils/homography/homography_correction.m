function [homographies, invhomographies, adjusted_gnd_detections, adjusted_ground_plane_regions, adjusted_overlap] = homography_correction(matchings, inplanes, ground_plane_regions, homog_solver, num_cams)
    if ~isempty(matchings{1}) && ~isempty(matchings{2}) % Can only correct if spatial correspondences
        % v_matchings = cell(2,1);
        % for i=1:length(cameras)
        %    if ~isempty(valid_matchings{i})
        %        v_matchings{i} = cell2mat(transpose(valid_matchings{i}));
        %    end
        % end
        fprintf('\t\t 11.Correcting homographies...\n');
        % TODO Find a way to pick best rho's
        [rho_r, rho_d, best_N] = determineRho(matchings, inplanes, ground_plane_regions, homog_solver); % Determines good rho values for convergence
        [homographies, adjusted_dets, adjusted_ground_plane_regions] = homography_algorithm(v_matchings, inplanes, ground_plane_regions, homog_solver, best_N, rho_r, rho_d, homocorrec_debug, min_delta);
        % NOTE Changed here from previous commit to simplify
        invhomographies = cell(num_cams,1);
        adjusted_gnd_detections = cell(num_cams,1);
        for i=1:num_cams
            invhomographies{i} = inv(homographies{i});
            adjusted_gnd_detections{i}{end+1} = adjusted_dets{i};
        end
        [adjusted_overlap, ~, ~] = computeOverlap(adjusted_ground_plane_regions);
    end
end
