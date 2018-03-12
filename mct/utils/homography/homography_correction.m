function [H1, H2, cam1_dets_gnd, cam2_dets_gnd, cam1_region_gnd, cam2_region_gnd] = homography_correction(matchings, inplanes, ground_plane_regions, homog_solver, N, rho_r, rho_d)
    %-----------------------------PREAMBLE---------------------------------
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
    %-----------------------------ITERATIVE PROCESS---------------------------------
    % DEBUG
    figure; subplot(2,2,1); hold on; title('Iterations converging on the ground plane.')
    n_c2 = cell(N+1,size(cam2_dets_gnd,2)); n_c1 = cell(N+1,size(cam1_dets_gnd,1));
    for i=1:size(cam2_dets_cam,1)
      n_c2{1,i} = cam2_dets_gnd_original(i,:);
      n_c1{1,i} = cam1_dets_gnd_original(i,:);
    end

    region_shifts = zeros(N,1); region1_shift = 0; region2_shift = 0;
    distances = zeros(N,1); dd = zeros(N-1,1);
    for reps = 1:N
        % "Fix" H1 and compute new H2
        regdet_mat1 = vertcat(repmat(cam2_region_cam,rho_r,1), repmat(cam2_dets_cam,rho_d,1));
        regdet_mat2 = vertcat(repmat(cam2_region_gnd,rho_r,1), repmat(cam1_dets_gnd,rho_d,1));
        H2 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

        % "Fix" H2 and compute new H1
        regdet_mat1 = vertcat(repmat(cam1_region_cam,rho_r,1), repmat(cam1_dets_cam,rho_d,1));
        regdet_mat2 = vertcat(repmat(cam1_region_gnd,rho_r,1), repmat(cam2_dets_gnd,rho_d,1));
        H1 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

        % Compute new cam2 ground plane regions and detections with n_H2
        %cam2_dets_gnd = H(n_H2,cam2_dets_cam);
        % NOTE I changed this from column wise to row wise
        for i=1:size(cam2_dets_cam,1)
          cam2_dets_gnd(i,:) = H(H2,transpose(cam2_dets_cam(i,:)));
          n_c2{reps+1,i} = cam2_dets_gnd(i,:);
        end
        %NOTE: There was a problem here with the clockwise ordenation of points, it made the method not converge
        cam2_region_gnd = reg2gnd(cam2_region_cam, H2);

        % Compute new cam1 ground plane regions and detections with n_H1
        %cam1_dets_gnd = H(n_H1,cam1_dets_cam);
        for i=1:size(cam1_dets_cam,1)
          cam1_dets_gnd(i,:) = H(H1,transpose(cam1_dets_cam(i,:)));
          n_c1{reps+1,i} = cam1_dets_gnd(i,:);
        end
        cam1_region_gnd = reg2gnd(cam1_region_cam, H1);
        %DEBUG
        drawPoly(cam1_region_gnd,'Yellow',0.5,false); % Draw region
        drawPoly(cam2_region_gnd,'Pink',0.5,false); % Draw region
        scatter(cam1_dets_gnd(:,1),cam1_dets_gnd(:,2),'MarkerFaceColor',rgb('Yellow'),'MarkerEdgeColor',rgb('Yellow'));
        scatter(cam2_dets_gnd(:,1),cam2_dets_gnd(:,2),'MarkerFaceColor',rgb('Pink'),'MarkerEdgeColor',rgb('Pink'));
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
        fprintf(['\t\t Iteration ', num2str(reps),'. Distance between ground plane detections: ', ...
          num2str(distances(reps)), '\n']);
        if reps ~= 1
            dd(reps-1) = distances(reps-1) - distances(reps);
        end
    end

    hold on;
    for i = 1:size(cam1_dets_gnd,1)
        n_s1 = n_c1(:,i);
        n_s2 = n_c2(:,i);
        ns1 = cell2mat(n_s1);
        ns2 = cell2mat(n_s2);
        plot(ns1(:,1),ns1(:,2),'k');
        plot(ns2(:,1),ns2(:,2),'k');
    end

    drawPoly(cam1_region_gnd,'Orange',0.5,false); % Draw region
    drawPoly(cam2_region_gnd,'Purple',0.5,false); % Draw region
    scatter(cam1_dets_gnd_original(:,1),cam1_dets_gnd_original(:,2),'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange'));
    scatter(cam2_dets_gnd_original(:,1),cam2_dets_gnd_original(:,2),'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple'));
    xlabel('x') % x-axis label
    ylabel('y') % y-axis label

    subplot(2,2,2);
    plot(1:N,distances);
    title('Distance between adjusted matchings.')
    xlabel('N') % x-axis label
    ylabel('distance') % y-axis label

    subplot(2,2,3);
    plot(1:N,region_shifts,'r');
    title('Avg distance shifts of camera regions.')
    xlabel('N') % x-axis label
    ylabel('shift') % y-axis label

    subplot(2,2,4);
    plot(1:(N-1),dd,'g');
    title('Derivative of adjustments.')
    xlabel('N') % x-axis label
    ylabel('d(distance)/dN') % y-axis label
end

%function reg = cw(in_reg) % Clockwise ordering of the points (works for convex polygons)
%    x = in_reg(:,1);
%    y = in_reg(:,2);
%    cx = mean(x);
%    cy = mean(y);
%    a = atan2(y - cy, x - cx);
%    [~, order] = sort(a);
%    x = x(order);
%    y = y(order);
%    reg = in_reg;
%    reg(:,1) = x;
%    reg(:,2) = y;
%end

function gpreg = reg2gnd(in_reg, h)
    s = size(in_reg,1);
    gpreg = zeros(s,2);
    for p=1:s
        pts = in_reg(p,:);
        gpreg(p,:) = H(h,pts');
    end
end
