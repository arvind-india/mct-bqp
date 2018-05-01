function [best_rho, best_N, N] = determineRho(matchings, inplanes, ground_plane_regions, homog_solver)
    % We fix rho_r at 1
    rho_r = 1;

    rhos = [1, 2, 4, 8, 10, 15, 20, 50, 100, 150];
    Ns = [10, 50, 100]; % Number of iterations
    best_rho = -1;
    best_N = -1;
    end_distance = 1000;
    % ---------------------

    cam1_region_cam = inplanes{1};
    cam2_region_cam = inplanes{2};
    cam1_region_gnd = ground_plane_regions{1}(1:end-1,:);
    cam2_region_gnd = ground_plane_regions{2}(1:end-1,:);
    cam1_dets_cam = [matchings{1}(:,4) + matchings{1}(:,6)./2 matchings{1}(:,5) + matchings{1}(:,7)];
    cam2_dets_cam = [matchings{2}(:,4) + matchings{2}(:,6)./2 matchings{2}(:,5) + matchings{2}(:,7)];
    cam1_dets_gnd = matchings{1}(:,8:9);
    cam2_dets_gnd = matchings{2}(:,8:9);
    cam1_region_gnd_original = cam1_region_gnd;
    cam2_region_gnd_original = cam2_region_gnd;
    cam1_dets_gnd_original = cam1_dets_gnd;
    cam2_dets_gnd_original = cam2_dets_gnd;
    for rho_idx = 1:length(rhos)
        rho_d = rhos(rho_idx);
        for n_idx = 1:length(Ns)
            N = Ns(n_idx);
            n_r2 = cell(N+1,1); n_r1 = cell(N+1,1);
            n_c2 = cell(N+1,size(cam2_dets_gnd,2)); n_c1 = cell(N+1,size(cam1_dets_gnd,1));
            for i=1:size(cam2_dets_cam,1)
              n_c2{1,i} = cam2_dets_gnd_original(i,:);
              n_c1{1,i} = cam1_dets_gnd_original(i,:);
            end
            region_shifts = zeros(N,1); region1_shift = 0; region2_shift = 0;
            distances = zeros(N,1);
            dd = zeros(N-1,1);
            distances_1 = zeros(N,1);
            distances_2 = zeros(N,1);

            for reps = 1:N
                % "Fix" H1 and compute new H2
                regdet_mat1 = vertcat(repmat(cam2_region_cam,rho_r,1), repmat(cam2_dets_cam,rho_d,1));
                regdet_mat2 = vertcat(repmat(cam2_region_gnd,rho_r,1), repmat(cam1_dets_gnd + noise_mat_d,rho_d,1));
                H2 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

                % "Fix" H2 and compute new H1
                regdet_mat1 = vertcat(repmat(cam1_region_cam,rho_r,1), repmat(cam1_dets_cam,rho_d,1));
                regdet_mat2 = vertcat(repmat(cam1_region_gnd + noise_mat_r,rho_r,1), repmat(cam2_dets_gnd + noise_mat_d,rho_d,1));
                H1 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

                % Compute new cam2 ground plane regions and detections with n_H2
                % NOTE I changed this from column wise to row wise
                for i=1:size(cam2_dets_cam,1)
                  cam2_dets_gnd(i,:) = H(H2,transpose(cam2_dets_cam(i,:)));
                  n_c2{reps+1,i} = cam2_dets_gnd(i,:);
                end
                %NOTE: There was a problem here with the clockwise ordenation of points, it made the method not converge
                cam2_region_gnd = reg2gnd(cam2_region_cam, H2);
                n_r2{reps+1} = cam2_region_gnd;
                % Compute new cam1 ground plane regions and detections with n_H1
                for i=1:size(cam1_dets_cam,1)
                  cam1_dets_gnd(i,:) = H(H1,transpose(cam1_dets_cam(i,:)));
                  n_c1{reps+1,i} = cam1_dets_gnd(i,:);
                end
                cam1_region_gnd = reg2gnd(cam1_region_cam, H1);
                n_r1{reps+1} = cam1_region_gnd;
                distances_1(reps) = pdist([cam2_dets_gnd(1,:); cam1_dets_gnd(1,:)]);
                distances_2(reps) = pdist([cam2_dets_gnd(2,:); cam1_dets_gnd(2,:)]);
                for i = 1:size(cam1_dets_gnd,1)
                    distances(reps) = distances(reps) + pdist([cam2_dets_gnd(i,:); cam1_dets_gnd(i,:)]);
                end
                for i1=1:size(cam1_region_gnd,1)
                    region1_shift = region1_shift + pdist([cam1_region_gnd(i,:); cam1_region_gnd_original(i,:)]);
                end
                region1_shift = region1_shift / i1;
                for i2=1:size(cam2_region_gnd)
                    region2_shift = region2_shift + pdist([cam2_region_gnd(i,:); cam2_region_gnd_original(i,:)]);
                end
                region2_shift = region2_shift / i2;
                region_shifts(reps) = (region1_shift + region2_shift)/2;
                region1_shift = 0; region2_shift = 0;
                fprintf(['\t\t' num2str(distances(reps)) ' -- Iter ', num2str(reps),'| dist between gnd plane detections: \n']);
                if reps ~= 1
                    dd(reps-1) = distances(reps-1) - distances(reps);
                end
                if dd(reps-1) < min_delta
                    break;
                end
            end

            if distances(reps) < end_distance
                end_distance = distances(reps);
                best_N = N;
                best_rho = rho_d;
            end
        end
    end

end
