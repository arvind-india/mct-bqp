% Create fictional camera detections for 2 cameras
%cam1_camdetections = [400.0; 700.0];
cam1_camdetections = [400.0; 700.0];
cam1_region = [4 796; 1022 798; 353 473; 142 431];

cam2_camdetections = [560.0; 640.0];
cam2_region = [59 796; 503 353; 1015 375; 1011 798];

% These two are the HDA+ elevator patio initial homographies
H1 = [0.043383 0.043388 35.129;
     -0.035325 0.022121	68.004;
     0.0020342 0.019584	1];
H2 = [0.053286 0.10144 -37.736;
      0.017139 0.0081285 45.967;
      0.0016208	0.016526 1];

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
cam1_gpdetections = H(H1,cam1_camdetections);
cam2_gpdetections = H(H2,cam2_camdetections);
scatter(cam1_gpdetections(1),cam1_gpdetections(2),'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red'));
scatter(cam2_gpdetections(1),cam2_gpdetections(2),'MarkerFaceColor',rgb('Blue'),'MarkerEdgeColor',rgb('Blue'));
% We would solve an assignment problem here...

%-----------------------------ITERATIVE PROCESS---------------------------------
rho_r = 4;
rho_m = 1;

% Initialize
n_cam2_gpregion = cam2_gpregion; n_cam2_gpdetections = cam2_gpdetections;
n_cam1_gpregion = cam1_gpregion; n_cam1_gpdetections = cam1_gpdetections;
% Iterate
N = 9;
homog_solver = 'svd';
n_c2 = zeros(N+1,2);
n_c2(1,:) = n_cam2_gpdetections';
n_c1 = zeros(N+1,2);
n_c1(1,:) = n_cam1_gpdetections';
power = -100;
distances = zeros(N,1);
tic
for reps = 1:N
    % "Fix" H1 and compute new H2 with some noise
    W_r = wgn(size(n_cam2_gpregion,1),size(n_cam2_gpregion,2),power);
    W_d = wgn(size(n_cam1_gpdetections,2),size(n_cam1_gpdetections,1),power);
    regdet_mat1 = vertcat(repmat(cam2_region,rho_r,1), repmat(cam2_camdetections',rho_m,1));
    regdet_mat2 = vertcat(repmat(n_cam2_gpregion,rho_r,1), repmat(n_cam1_gpdetections',rho_m,1));
    n_H2 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

    % Compute new cam2 ground plane regions and detections with n_H2
    %n_cam2_gpdetections = H(n_H2,cam2_camdetections); n_c2(reps+1,:) = n_cam2_gpdetections';
    %n_cam2_gpregion = cw(reg2gnd(cam2_region, n_H2));

    % "Fix" H2 and compute H1 with some noise
    W_r = wgn(size(n_cam1_gpregion,1),size(n_cam1_gpregion,2),power);
    W_d = wgn(size(n_cam2_gpdetections,2),size(n_cam2_gpdetections,1),power);
    regdet_mat1 = vertcat(repmat(cam1_region,rho_r,1), repmat(cam1_camdetections',rho_m,1));
    regdet_mat2 = vertcat(repmat(n_cam1_gpregion, rho_r,1), repmat(n_cam2_gpdetections',rho_m,1));
    n_H1 = solve_homography(regdet_mat1, regdet_mat2, homog_solver);

    % Compute new cam2 ground plane regions and detections with n_H2
    n_cam2_gpdetections = H(n_H2,cam2_camdetections);
    n_c2(reps+1,:) = n_cam2_gpdetections';
    %NOTE: There was a problem here with the clockwise ordenation of points, it made the method not converge
    n_cam2_gpregion = reg2gnd(cam2_region, n_H2);

    % Compute new cam1 ground plane regions and detections with n_H1
    n_cam1_gpdetections = H(n_H1,cam1_camdetections);
    n_c1(reps+1,:) = n_cam1_gpdetections';
    n_cam1_gpregion = cw(reg2gnd(cam1_region, n_H1));

    drawPoly(n_cam1_gpregion,'Yellow',0.5,false); % Draw region
    drawPoly(n_cam2_gpregion,'Pink',0.5,false); % Draw region
    scatter(n_cam1_gpdetections(1),n_cam1_gpdetections(2),'MarkerFaceColor',rgb('Yellow'),'MarkerEdgeColor',rgb('Yellow'));
    scatter(n_cam2_gpdetections(1),n_cam2_gpdetections(2),'MarkerFaceColor',rgb('Pink'),'MarkerEdgeColor',rgb('Pink'));
    distances(reps) = sqrt((n_cam2_gpdetections(1)-n_cam1_gpdetections(1))^2 + (n_cam2_gpdetections(2)-n_cam1_gpdetections(2))^2);
    fprintf(['\t Iteration ', num2str(reps),'. Distance between ground plane detections: ', ...
      num2str(distances(reps)), '\n']);
end
time=toc;
fprintf(['Loop took: ', num2str(round(time*100)/100), '\n']);
hold on
plot(n_c2(:,1),n_c2(:,2),'k'); plot(n_c1(:,1),n_c1(:,2),'k'); % Draw iteration process trace
drawPoly(n_cam1_gpregion,'Orange',0.5,false); % Draw region
drawPoly(n_cam2_gpregion,'Purple',0.5,false); % Draw region
scatter(n_cam1_gpdetections(1),n_cam1_gpdetections(2),'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange'));
scatter(n_cam2_gpdetections(1),n_cam2_gpdetections(2),'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple'));
subplot(1,2,2)
plot(1:N,distances);
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

function t = H(matrix, point)
    t = matrix * [point; 1];
    t(1) = t(1)/t(3);
    t(2) = t(2)/t(3);
    t = t(1:2);
end
