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
cam_imgs = cell(2,1);
cam_imgs{1} = imread('1147.png'); cam_imgs{2} = imread('1017.png');
% List of peds in the two cameras
% cam frame x y width height bit ped
% 2 peds - camera 57 -- allDetections{1}{1147}
cam_peds = cell(2,1);
cam_peds{1} = [57 1147 108 161 262 591 1 15 ; ...
              57 1147 559 159 271 637 1 52];
% 3 peds - camera 58 -- allDetections{2}{1017}
% The third pedestrian, ped52 actually wasn't a valid detection since their feet are not visible, but lets put him in for the sake of the experiment
cam_peds{2} = [58 1017 3 250 325 542 1 23; ...
              58 1017 575 144 198 608 1 15; ...
              58 1017 275 295 336 501 1 52];

ped_ids = [15, 52, 23];
ped_colors = {'Red', 'Cyan', 'Green'};

% Convert positions to the ground plane and draw Histograms simultaneously
cam_peds_gnd = cell(2,1);
for i = 1:2
    cam_peds_gnd{i} = drawHistograms_computeGndDetections(cam_peds{i}, cam_imgs{i}, homographies{i});
end

figure; hold on;
%openfig('~/mct-bqp/hda_data/homographies/7th_floor_ground_plane_reference_frame_map.fig');
for p = 1:length(ped_ids)
    for i=1:size(cam_1_peds_gnd,1)
        for c = 1:2
            if cam_peds{c}(i,8) == ped_ids(p)
                scatter(cam_peds_gnd{c}(i,1),cam_peds_gnd{c}(i,2),'MarkerFaceColor',rgb(ped_colors{p}),'MarkerEdgeColor',rgb(ped_colors{p}));
            end
        end
    end
end

% TODO Solve spatial assignment using Frank-Wolfe

% Preamble -- we assume all the targets are in the overlap for simplicity
n_o = cell(2,1);
for i = 1:2
    n_o{i} = size(cam_peds{i},1);
end
N_o = sum(cell2mat(n_o));

for i = 1:2
    cands_spatial = cell(n_o{i},1);
    other_cam = rem(i,2) + 1; % TODO This is a dirty hack
    c_spatial = cam_peds{other_cam}; % Basically, the candidates are the pedestrians in the other cameras
    for j = 1:n_o{i}
        cands_spatial{j} = cam_peds_gnd{i}(j,:);
    end
end

% TODO Create fictional appearance cues
a = cell(2,1);
weights = cell(2,1);
Z_models = cell(2,1); y_models = cell(2,1);
for i = 1:2

    % TODO Create models
    [Zs, ys] = appearance_model(n,targs,image,a_sigma,dx,dy,g_candidates);

    % TODO store models
    Z_models{i} = Zs; y_models{i} = ys;

    % TODO Compute apperance using this model
    [c_a, w] = appearance(k,n,cands_percam{i},next_images{i},appearance_method,lambda,weights{i},filter,Zs,ys);
    a{i} = c_a;

    % TODO Store weights and use them in a filter-like implementation
    weights{i} = w;
end
a = normalize_spatial(a,k,N); % NOTE Normalize across each candidate

% ---------------------------------------------------------------
f = 1; detection_frames = [1, 4, 6];
% TODO Create fictional motion cues
if ismember(f,detection_frames)
    fprintf('\t 7.Creating motion models...\n');
    mm = 1;
    motion_models = cell(N_o,1);
    for i = 1:2
        for t = 1:size(cands_spatial,1)
            initial_speed_x = 0.0;
            %initial_speed_x = sum(gnd_detections{cam}{start_frames(cam) + detection_frames(2)}(:,8) - gnd_detections{cam}{start_frames(cam) + detection_frames(1)}(:,8)./dt);
            initial_speed_y = 1.0;
            %initial_speed_y = sum(gnd_detections{cam}{start_frames(cam) + detection_frames(2)}(:,9) - gnd_detections{cam}{start_frames(cam) + detection_frames(1)}(:,9)./dt);

            motion_models{mm} = [cands_spatial{t}(1); cands_spatial{t}(2); initial_speed_x; initial_speed_y]; % Assign the new models
            mm = mm + 1;
        end
    end
end
% NOTE motion
fprintf('\t 8.Computing motion cues...\n');
m = cell(length(cameras),1);
m_sigma{1} = [0.5 0; 0 0.5]; m_sigma{2} = [0.5 0; 0 0.5];
for i = 1:2
    other_cam = rem(i,2) + 1;
    c_m = motion(n_o{i},n_o{other_cam},motion_models,cands_spatial{i},fps,m_sigma{i});
    m{i} = c_m;
    %plotMotion(i, c_m, k, n, floor_image, cands_percam);
end
m = normalize_spatial(m,k,N); % NOTE Normalize across each candidate!

% ---------------------------------------------------------------

% TODO Create fictional grouping cues

% TODO Create variables for FW

% TODO Run FW

% TODO Parse results

time = toc;
fprintf(['Time: ', num2str(round(time*100)/100), ' seconds \n']);
fprintf('Optimal assignment found: \n');
for i=1:size(S,1)
  fprintf('Best assignment for ped %d in cam %d === ped %d in cam %d \n', cam_1_peds(i,8), cam_1_peds(i,1),...
  cam_2_peds(assignments(i),8), cam_2_peds(assignments(i),1));
end

function normalized = normalize_spatial(array, n_o)
    for i = 1:2
        % Get targets
        % Get candidates
        for each target
            normalize along its candidates

    end

end


function t = H(matrix, point)
    t = matrix * [point; 1];
    t(1) = t(1)/t(3);
    t(2) = t(2)/t(3);
    t = t(1:2);
end

function cam_peds_gnd = drawHistograms_computeGndDetections(peds_data, cam_img, h_matrix)
  n = size(peds_data, 1);
  cam_peds_gnd = zeros(n,2);
  rgb_histograms = cell(n,1);
  figure
  hold on
  for i=1:n
    cam_peds_gnd(i,:) = H(h_matrix,[peds_data(i,3) + peds_data(i,5)/2; peds_data(i,4) + peds_data(i,6)/2]);
    bb_img = imcrop(cam_img,peds_data(i,3:6));
    I = [imhist(bb_img(:,:,1)) imhist(bb_img(:,:,2)) imhist(bb_img(:,:,3))];
    rgb_histograms{i} = I;
    for j=1:3
      subplot(n,3,j+3*(i-1))
      if j == 1
        bar(rgb_histograms{i}(:,j), 'r')
        title(['Ped:', num2str(peds_data(i,8)), ' R']);
      elseif j == 2
        bar(rgb_histograms{i}(:,j), 'g')
        title(['Ped:', num2str(peds_data(i,8)), ' G']);
      elseif j == 3
        bar(rgb_histograms{i}(:,j), 'b')
        title(['Ped:', num2str(peds_data(i,8)), ' B']);
      end
    end
  end
end
