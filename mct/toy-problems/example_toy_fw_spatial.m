% Fictional asymetric assignment with 2 cameras, one with 3 people and the other with 2 people
tic
homographies = cell(2,1);
homographies{1} = [0.043383 0.043388 35.129;
     -0.035325 0.022121	68.004;
     0.0020342 0.019584	1];
homographies{2} = [0.053286 0.10144 -37.736;
      0.017139 0.0081285 45.967;
      0.0016208	0.016526 1];

% Load images of the same frame in the two images
images = cell(2,1);
images{1} = imread('1147.png'); images{2} = imread('1017.png');
next_images{1} = imread('1148.png'); next_images{2} = imread('1018.png');
future_images{1} = imread('1153.png'); future_images{2} = imread('1023.png');
% List of targs in the two cameras
% cam frame x y width height bit ped
cameras = [57 58];
% 2 targs - camera 57 -- allDetections{1}{1147}
cam_targs = cell(2,1);
cam_targs{1} = [57 15 1147 108 161 262 591; ...
              57 52 1147 559 159 271 637];
% 3 targs - camera 58 -- allDetections{2}{1017}
% The third pedestrian, ped52 actually wasn't a valid detection since their feet are not visible, but lets put him in for the sake of the experiment
cam_targs{2} = [58 23 1017 3 250 325 542; ...
              58 15 1017 575 144 198 608; ...
              58 52 1017 275 295 336 501];

% Target positions after 4 frames
cam_targs_future{1} = [57 15 1147 108 161 262 591; ...
              57 52 1147 559 159 271 637];
cam_targs_future{2} = [58 23 1017 3 250 325 542; ...
              58 15 1017 575 144 198 608; ...
              58 52 1017 275 295 336 501];

ped_ids = [15, 52, 23];
ped_colors = {'Red', 'Cyan', 'Green'};

% Convert positions to the ground plane and draw Histograms simultaneously
cam_targs_gnd = cell(2,1); cam_targs_gnd_future = cell(2,1);
for i = 1:2
    cam_targs_gnd_future{i} = computeGndDetections(cam_targs_future{i}, homographies{i});
    cam_targs_gnd{i} = computeGndDetections(cam_targs{i}, homographies{i});
    drawHistograms(cam_targs_future{i}, future_images{i});
    %drawHistograms(cam_targs{i}, images{i});
end

%openfig('~/mct-bqp/hda_data/regions/7th_floor_ground_plane_reference_frame_map.fig');
figure;
hold on;
for p = 1:length(ped_ids)
    for c = 1:2
        for i=1:size(cam_targs{c},1)
            if cam_targs{c}(i,2) == ped_ids(p)
                scatter(cam_targs_gnd{c}(i,1),cam_targs_gnd{c}(i,2),'MarkerFaceColor',rgb(ped_colors{p}),'MarkerEdgeColor',rgb(ped_colors{p}));
            end
        end
    end
end

% TODO Solve spatial assignment using Frank-Wolfe
% Preamble -- we assume all the targets are in the overlap for simplicity
n_o = cell(2,1);
for i = 1:2
    n_o{i} = size(cam_targs{i},1);
end
N_o = sum(cell2mat(n_o));

cands_spatial = cell(2,1);
for i = 1:2
    cands_spatial{i} = cell(n_o{i},1);
    other_cam = rem(i,2) + 1; % TODO This is a dirty hack
    c_spatial = cam_targs{other_cam}; % Basically, the candidates are the targets in the other cameras
    for j=1:n_o{i}
        %cands_spatial{i}{j} = [c_spatial(:,4:7) c_spatial(:,2)];
        cands_spatial{i}{j} = [c_spatial(:,4:7) cam_targs_gnd{other_cam}];
    end
end

% TODO Create fictional appearance cues
a = cell(2,1);
weights = cell(2,1);
Z_models = cell(2,1); y_models = cell(2,1);
a_sigma{1} = [1 ^ 2 0.0; 0.0 1 ^ 2]; a_sigma{2} = [1 ^ 2 0.0; 0.0 1 ^ 2];
dx = 10; dy = 50; % Sampling in the camera space
g_candidates = 9;
appearance_method = 'naive';
lambda = 0.1;
filter = 'none';
figure;
for i = 1:2
    fprintf('\t Apperance model ... \n');
    % TODO Create models
    [Zs, ys] = appearance_model(n_o{i},cam_targs{i},images{i},a_sigma{i},dx,dy,g_candidates, cameras);

    % TODO store models
    Z_models{i} = Zs; y_models{i} = ys;

    fprintf('\t Apperance cues ... \n');
    % TODO Compute apperance using this model
    k = size(cands_spatial{i}{1},1); % Use 1 or any other, they should have the same size
    [c_a, w] = appearance(k,n_o{i},cands_spatial{i},next_images{i},appearance_method,lambda,weights{i},filter,Zs,ys);
    a{i} = c_a;

    % TODO Store weights and use them in a filter-like implementation
    weights{i} = w;
end
plotAppearanceWeights(weights);
a = cell2mat(a);
%a = normalize_spatial(a,k,N); % NOTE Normalize across each candidate

% ---------------------------------------------------------------
% TODO Create fictional motion cues
fprintf('\t Motion models ...  \n');
mm = 1;
fps = 2;
dt = 1/fps;
motion_models = cell(N_o,1);
for i = 1:2
    for t = 1:size(cam_targs_gnd,1)
        %initial_speed_x = 0.0;
        initial_speed_x = (cam_targs_gnd_future{i}(t,1) - cam_targs_gnd{i}(t,1))/dt;
        %initial_speed_y = 1.0;
        initial_speed_y = (cam_targs_gnd_future{i}(t,2) - cam_targs_gnd{i}(t,2))/dt;

        motion_models{mm} = [cam_targs_gnd{i}(t,1); cam_targs_gnd{i}(t,2); initial_speed_x; initial_speed_y]; % Assign the new models
        mm = mm + 1;
    end
end
% NOTE motion
fprintf('\t Motion cues ... \n');
m = cell(length(cameras),1);
m_sigma{1} = [0.5 0; 0 0.5]; m_sigma{2} = [0.5 0; 0 0.5];
for i = 1:2
    other_cam = rem(i,2) + 1;
    c_m = motion(n_o{i},n_o{other_cam},motion_models,cands_spatial{i},fps,m_sigma{i});
    m{i} = c_m;
    %plotMotion(i, c_m, k, n, floor_image, cands_percam);
end
%m = normalize_spatial(m,k,N); % NOTE Normalize across each candidate!
m = cell2mat(m);

fprintf('\t Spatial grouping ... \n')
% TODO Create fictional grouping cues
G = zeros(N*k);
T_spatial = h_grouping(N_o,targs_o,targs_in_overlap,groups_spatial,h_G_sigma);
Dinvsq_spatial = diag(sum(T_spatial,2)).^(-1/2); %row sum
Dinvsq_spatial(~isfinite(Dinvsq_spatial)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
G_spatial = Dinvsq_spatial*T_spatial*Dinvsq_spatial; %TODO Due to its structure it cant be Normalized Laplacian matrix so G is convex
%G_spatial = zeros(N_o*2,N_o*2);

% TODO Create variables for FW

% TODO Run FW

% TODO Parse results

kill;

time = toc;
fprintf(['Time: ', num2str(round(time*100)/100), ' seconds \n']);
fprintf('Optimal assignment found: \n');
for i=1:size(S,1)
  fprintf('Best assignment for ped %d in cam %d === ped %d in cam %d \n', cam_1_targs(i,8), cam_1_targs(i,1),...
  cam_2_targs(assignments(i),8), cam_2_targs(assignments(i),1));
end

%function normalized = normalize_spatial(array, n_o)
%    for i = 1:2
        % Get targets
        % Get candidates
%        for each target
%            normalize along its candidates
%
%    end
%
%end

function t = H(matrix, point)
    t = matrix * [point; 1];
    t(1) = t(1)/t(3);
    t(2) = t(2)/t(3);
    t = t(1:2);
end

function cam_targs_gnd = computeGndDetections(targs_data, h_matrix)
  n = size(targs_data, 1);
  cam_targs_gnd = zeros(n,2);
  for i=1:n
    cam_targs_gnd(i,:) = H(h_matrix,[targs_data(i,4) + targs_data(i,6)/2; targs_data(i,5) + targs_data(i,7)/2]);
  end
end

function drawHistograms(targs_data, cam_img)
    n = size(targs_data, 1);
    rgb_histograms = cell(n,1);
    figure
    hold on
    for i=1:n
      bb_img = imcrop(cam_img,targs_data(i,4:7));
      I = [imhist(bb_img(:,:,1)) imhist(bb_img(:,:,2)) imhist(bb_img(:,:,3))];
      rgb_histograms{i} = I;
      for j=1:3
        subplot(n,3,j+3*(i-1))
        if j == 1
          bar(rgb_histograms{i}(:,j), 'r')
          title(['Ped:', num2str(targs_data(i,2)), ' R']);
        elseif j == 2
          bar(rgb_histograms{i}(:,j), 'g')
          title(['Ped:', num2str(targs_data(i,2)), ' G']);
        elseif j == 3
          bar(rgb_histograms{i}(:,j), 'b')
          title(['Ped:', num2str(targs_data(i,2)), ' B']);
        end
      end
    end
end
