function [n_H1, n_H2] = correct_homographies(H1,H2,cam1_camdetections, cam1_region, cam2_camdetections, cam2_region)
  figure;
  subplot(1,2,1)
   hold on;
  cam1_gpregion = reg2gnd(cam1_region, H1);
  drawPoly(cam1_gpregion,'Red',0.8,false); % Draw region
  cam2_gpregion = reg2gnd(cam2_region, H2);
  drawPoly(cam2_gpregion,'Blue',0.8,false); % Draw region
  ground_plane_regions = {};
  ground_plane_regions{1} = cam1_gpregion;
  ground_plane_regions{2} = cam2_gpregion;
  % Plot overlap of camera regions
  [overlap, ~, ~] = computeOverlap(ground_plane_regions); % Performed in the pixel plane
  drawPoly(overlap,'Black',1.0,false);
  %------------------------------------------------------------------------------
  % Compute initial detection in the ground plane
  cam1_gpdetections = cam1_camdetections; cam2_gpdetections = cam2_camdetections;

  if size(cam1_camdetections,2) == size(cam2_camdetections,2)
    for i=1:size(cam1_camdetections,2)
      v1 = H(H1,cam1_camdetections(:,i));
      cam1_gpdetections(:,i) = v1;
      v2 = H(H2,cam2_camdetections(:,i));
      cam2_gpdetections(:,i) = v2;
      scatter(cam1_gpdetections(1,i),cam1_gpdetections(2,i),'MarkerFaceColor',rgb('Black'),'MarkerEdgeColor',rgb('Black'));
      scatter(cam2_gpdetections(1,i),cam2_gpdetections(2,i),'MarkerFaceColor',rgb('White'),'MarkerEdgeColor',rgb('Black'));
      % TODO this is a nice case scenario, a stricter imposition should be that they are in the overlap region (sometimes this not for the best)
      if polyin(v1',cam1_gpregion) == 0 || polyin(v2',cam2_gpregion) == 0
        cam2_gpdetections(:,i) = [];
        cam1_gpdetections(:,i) = [];
        cam2_camdetections(:,i) = [];
        cam1_camdetections(:,i) = [];
      else
        cam2_gpdetections(:,i) = v2;
        cam1_gpdetections(:,i) = v1;
      end
    end
  end
  scatter(cam1_gpdetections(1,:),cam1_gpdetections(2,:),'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red'));
  scatter(cam2_gpdetections(1,:),cam2_gpdetections(2,:),'MarkerFaceColor',rgb('Blue'),'MarkerEdgeColor',rgb('Blue'));

  %-----------------------------ITERATIVE PROCESS---------------------------------
  rho_r = 8;
  rho_m = 1;

  % Initialize
  n_cam2_gpregion = cam2_gpregion; n_cam2_gpdetections = cam2_gpdetections;
  n_cam1_gpregion = cam1_gpregion; n_cam1_gpdetections = cam1_gpdetections;
  % Iterate
  option = 'iterations'; % Pick 'iterations' or 'delta'
  delta = 0.01;
  N = 9;
  homog_solver = 'svd';
  n_c2 = zeros(N+1,2);
  n_c2(1,:) = n_cam2_gpdetections(:,1)';
  n_c1 = zeros(N+1,2);
  n_c1(1,:) = n_cam1_gpdetections(:,1)';
  power = -100;
  distances = zeros(N,1);
  tic
  if strcmp('iterations', option)
    for reps = 1:N
        % "Fix" H1 and compute new H2 with some noise
        W_r = wgn(size(n_cam2_gpregion,1),size(n_cam2_gpregion,2),power);
        W_d = wgn(size(n_cam1_gpdetections,2),size(n_cam1_gpdetections,1),power);
        regdet_mat1 = vertcat(repmat(cam2_region,rho_r,1), repmat(cam2_camdetections',rho_m,1));
        regdet_mat2 = vertcat(repmat(n_cam2_gpregion + W_r,rho_r,1), repmat(n_cam1_gpdetections' + W_d,rho_m,1));
        n_H2 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

        % "Fix" H2 and compute H1 with some noise
        W_r = wgn(size(n_cam1_gpregion,1),size(n_cam1_gpregion,2),power);
        W_d = wgn(size(n_cam2_gpdetections,2),size(n_cam2_gpdetections,1),power);
        regdet_mat1 = vertcat(repmat(cam1_region,rho_r,1), repmat(cam1_camdetections',rho_m,1));
        regdet_mat2 = vertcat(repmat(n_cam1_gpregion + W_r, rho_r,1), repmat(n_cam2_gpdetections' + W_d,rho_m,1));
        n_H1 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

        % Compute new cam2 ground plane regions and detections with n_H2
        %n_cam2_gpdetections = H(n_H2,cam2_camdetections);
        for i=1:size(cam2_camdetections,2)
          n_cam2_gpdetections(:,i) = H(n_H2,cam2_camdetections(:,i));
        end
        n_c2(reps+1,:) = n_cam2_gpdetections(:,1)';
        %NOTE: There was a problem here with the clockwise ordenation of points, it made the method not converge
        n_cam2_gpregion = reg2gnd(cam2_region, n_H2);

        % Compute new cam1 ground plane regions and detections with n_H1
        %n_cam1_gpdetections = H(n_H1,cam1_camdetections);
        for i=1:size(cam1_camdetections,2)
          n_cam1_gpdetections(:,i) = H(n_H1,cam1_camdetections(:,i));
        end
        n_c1(reps+1,:) = n_cam1_gpdetections(:,1)';
        n_cam1_gpregion = cw(reg2gnd(cam1_region, n_H1));

        drawPoly(n_cam1_gpregion,'Yellow',0.5,false); % Draw region
        drawPoly(n_cam2_gpregion,'Pink',0.5,false); % Draw region
        scatter(n_cam1_gpdetections(1),n_cam1_gpdetections(2),'MarkerFaceColor',rgb('Yellow'),'MarkerEdgeColor',rgb('Yellow'));
        scatter(n_cam2_gpdetections(1),n_cam2_gpdetections(2),'MarkerFaceColor',rgb('Pink'),'MarkerEdgeColor',rgb('Pink'));
        distances(reps) = sqrt((n_cam2_gpdetections(1)-n_cam1_gpdetections(1))^2 + (n_cam2_gpdetections(2)-n_cam1_gpdetections(2))^2);
        fprintf(['\t Iteration ', num2str(reps),'. Distance between ground plane detections: ', ...
          num2str(distances(reps)), '\n']);
    end
  elseif strcmp('delta', option)
    d = 0;
    while d < delta
        % "Fix" H1 and compute new H2 with some noise
        W_r = wgn(size(n_cam2_gpregion,1),size(n_cam2_gpregion,2),power);
        W_d = wgn(size(n_cam1_gpdetections,2),size(n_cam1_gpdetections,1),power);
        regdet_mat1 = vertcat(repmat(cam2_region,rho_r,1), repmat(cam2_camdetections',rho_m,1));
        regdet_mat2 = vertcat(repmat(n_cam2_gpregion,rho_r,1), repmat(n_cam1_gpdetections',rho_m,1));
        n_H2 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

        % "Fix" H2 and compute H1 with some noise
        W_r = wgn(size(n_cam1_gpregion,1),size(n_cam1_gpregion,2),power);
        W_d = wgn(size(n_cam2_gpdetections,2),size(n_cam2_gpdetections,1),power);
        regdet_mat1 = vertcat(repmat(cam1_region,rho_r,1), repmat(cam1_camdetections',rho_m,1));
        regdet_mat2 = vertcat(repmat(n_cam1_gpregion, rho_r,1), repmat(n_cam2_gpdetections',rho_m,1));
        n_H1 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

        % Compute new cam2 ground plane regions and detections with n_H2
        %n_cam2_gpdetections = H(n_H2,cam2_camdetections);
        for i=1:size(cam2_camdetections,2)
          n_cam2_gpdetections(:,i) = H(n_H2,cam2_camdetections(:,i))
        end
        n_c2(reps+1,:) = n_cam2_gpdetections';
        %NOTE: There was a problem here with the clockwise ordenation of points, it made the method not converge
        n_cam2_gpregion = reg2gnd(cam2_region, n_H2);

        % Compute new cam1 ground plane regions and detections with n_H1
        %n_cam1_gpdetections = H(n_H1,cam1_camdetections);
        for i=1:size(cam1_camdetections,2)
          n_cam1_gpdetections(:,i) = H(n_H1,cam1_camdetections(:,i))
        end
        n_c1(reps+1,:) = n_cam1_gpdetections';
        n_cam1_gpregion = cw(reg2gnd(cam1_region, n_H1));

        drawPoly(n_cam1_gpregion,'Yellow',0.5,false); % Draw region
        drawPoly(n_cam2_gpregion,'Pink',0.5,false); % Draw region
        scatter(n_cam1_gpdetections(1),n_cam1_gpdetections(2),'MarkerFaceColor',rgb('Yellow'),'MarkerEdgeColor',rgb('Yellow'));
        scatter(n_cam2_gpdetections(1),n_cam2_gpdetections(2),'MarkerFaceColor',rgb('Pink'),'MarkerEdgeColor',rgb('Pink'));
        d = sqrt((n_cam2_gpdetections(1)-n_cam1_gpdetections(1))^2 + (n_cam2_gpdetections(2)-n_cam1_gpdetections(2))^2);
        fprintf(['\t Distance between ground plane detections: ', num2str(d), '\n']);
    end
  end
  time=toc;
  fprintf(['Loop took (remember, cache makes a difference): ', num2str(round(time*100)/100), ' seconds \n']);
  hold on
  plot(n_c2(:,1),n_c2(:,2),'k'); plot(n_c1(:,1),n_c1(:,2),'k'); % Draw iteration process trace
  drawPoly(n_cam1_gpregion,'Orange',0.5,false); % Draw region
  drawPoly(n_cam2_gpregion,'Purple',0.5,false); % Draw region
  scatter(n_cam1_gpdetections(1),n_cam1_gpdetections(2),'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange'));
  scatter(n_cam2_gpdetections(1),n_cam2_gpdetections(2),'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple'));
  subplot(1,2,2)
  plot(1:N,distances);
end
%---------------------------------FUNCTIONS-------------------------------------
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
