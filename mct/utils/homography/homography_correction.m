function [H1, H2, regdet_mat1, regdet_mat2] = homography_correction(matchings, inplanes, ground_plane_regions)
    %-----------------------------PREAMBLE---------------------------------
    cam1_region_cam = inplanes{1};
    cam2_region_cam = inplanes{2};
    cam1_region_gnd = ground_plane_regions{1}(1:end-1,:);
    cam2_region_gnd = ground_plane_regions{2}(1:end-1,:);
    cam1_dets_cam = [matchings{1}(:,4)+matchings{1}(:,6)./2 matchings{1}(:,5)+matchings{1}(:,7)./2];
    cam2_dets_cam = [matchings{2}(:,4)+matchings{2}(:,6)./2 matchings{2}(:,5)+matchings{2}(:,7)./2];
    cam1_dets_gnd = matchings{1}(:,8:9);
    cam2_dets_gnd = matchings{2}(:,8:9);
    cam1_dets_gnd_original = cam1_dets_gnd;
    cam2_dets_gnd_original = cam2_dets_gnd;
    %-----------------------------ITERATIVE PROCESS---------------------------------
    % Iterate
    homog_solver = 'svd';
    N = 9;
    rho_r = 8;
    rho_m = 1;
    figure;
    subplot(1,2,1)
    hold on;
    n_c2 = zeros(N+1,2);
    n_c2(1,:) = cam2_dets_gnd(:,1)';
    n_c1 = zeros(N+1,2);
    n_c1(1,:) = cam1_dets_gnd(:,1)';
    distances = zeros(N,1);
    for reps = 1:N
        % "Fix" H1 and compute new H2
        regdet_mat1 = vertcat(repmat(cam2_region_cam,rho_r,1), repmat(cam2_dets_cam,rho_m,1));
        regdet_mat2 = vertcat(repmat(cam2_region_gnd,rho_r,1), repmat(cam1_dets_gnd,rho_m,1));
        H2 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

        % "Fix" H2 and compute new H1
        regdet_mat1 = vertcat(repmat(cam1_region_cam,rho_r,1), repmat(cam1_dets_cam,rho_m,1));
        regdet_mat2 = vertcat(repmat(cam1_region_gnd,rho_r,1), repmat(cam2_dets_gnd,rho_m,1));
        H1 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

        % Compute new cam2 ground plane regions and detections with n_H2
        %cam2_dets_gnd = H(n_H2,cam2_dets_cam);
        for i=1:size(cam2_dets_cam,2)
          cam2_dets_gnd(:,i) = H(H2,cam2_dets_cam(:,i));
        end
        n_c2(reps+1,:) = cam2_dets_gnd(:,1)';
        %NOTE: There was a problem here with the clockwise ordenation of points, it made the method not converge
        cam2_region_gnd = reg2gnd(cam2_region_cam, H2);

        % Compute new cam1 ground plane regions and detections with n_H1
        %cam1_dets_gnd = H(n_H1,cam1_dets_cam);
        for i=1:size(cam1_dets_cam,2)
          cam1_dets_gnd(:,i) = H(H1,cam1_dets_cam(:,i));
        end
        n_c1(reps+1,:) = cam1_dets_gnd(:,1)';
        cam1_region_gnd = cw(reg2gnd(cam1_region_cam, H1));

        drawPoly(cam1_region_gnd,'Yellow',0.5,false); % Draw region
        drawPoly(cam2_region_gnd,'Pink',0.5,false); % Draw region
        scatter(cam1_dets_gnd(1),cam1_dets_gnd(2),'MarkerFaceColor',rgb('Yellow'),'MarkerEdgeColor',rgb('Yellow'));
        scatter(cam2_dets_gnd(1),cam2_dets_gnd(2),'MarkerFaceColor',rgb('Pink'),'MarkerEdgeColor',rgb('Pink'));
        distances(reps) = sqrt((cam2_dets_gnd(1)-cam1_dets_gnd(1))^2 + (cam2_dets_gnd(2)-cam1_dets_gnd(2))^2);
        fprintf(['\t\t Iteration ', num2str(reps),'. Distance between ground plane detections: ', ...
          num2str(distances(reps)), '\n']);
    end

    hold on;
    plot(n_c2(:,1),n_c2(:,2),'k'); plot(n_c1(:,1),n_c1(:,2),'k'); % Draw iteration process trace
    drawPoly(cam1_region_gnd,'Orange',0.5,false); % Draw region
    drawPoly(cam2_region_gnd,'Purple',0.5,false); % Draw region
    scatter(cam1_dets_gnd_original(1),cam1_dets_gnd_original(2),'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange'));
    scatter(cam2_dets_gnd_original(1),cam2_dets_gnd_original(2),'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple'));
    subplot(1,2,2)
    plot(1:N,distances);
end

function reg = cw(in_reg) % Clockwise ordering of the points (works for convex polygons)
    x = in_reg(:,1);
    y = in_reg(:,2);
    cx = mean(x);
    cy = mean(y);
    a = atan2(y - cy, x - cx);
    [~, order] = sort(a);
    x = x(order);
    y = y(order);
    reg = in_reg;
    reg(:,1) = x;
    reg(:,2) = y;
end

function gpreg = reg2gnd(in_reg, h)
    s = size(in_reg,1);
    gpreg = zeros(s,2);
    for p=1:s
        pts = in_reg(p,:);
        n = h * [pts(1); pts(2); 1];
        n = [n(1)./n(3) n(2)./n(3)];
        gpreg(p,:) = n;
    end
end
