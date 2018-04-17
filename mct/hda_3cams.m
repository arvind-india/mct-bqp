set(0,'DefaultFigureVisible','on');
setDetectionParams_hda_hall_3cams; setTrackerParams;

gnd_detections = load_data('hda', cameras); % Load the images (needed for the appearance cues)
cameraListImages = cell(length(cameras),1); inplanes = cell(length(cameras),1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, durations(i), 0, 'hda');
    inplanes{i} = load(strcat(visibility_regions_directory,num2str(cameras{i}),'.mat')); inplanes{i} = inplanes{i}.t;
end
[homographies, invhomographies] = loadHomographies(homography_directory,'hda', cameras);
ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'hda');
[overlap, ~, ~] = computeOverlap(ground_plane_regions);
%%=========================================================
best_starts = zeros(size(gnd_detections{3},1),1);
for i=1:size(gnd_detections{3},1)
    if ~isempty(gnd_detections{3}{i}) && ~isempty(gnd_detections{2}{i+offset_frames(2)}) && ~isempty(gnd_detections{1}{i+offset_frames(1)})
        best_starts(i) = size(gnd_detections{3}{i},1) + size(gnd_detections{2}{i+offset_frames(2)},1) + size(gnd_detections{1}{i+offset_frames(1)},1);
    end
end

[start_max_cands, start] = max(best_starts);

%%=========================================================
num_frames = 7; % Number of frames
%start = 6811;
start_frames = [start + offset_frames(1) start + offset_frames(2) start + offset_frames(3)]; % Frames to start collecting images
g_candidates = 9; % basis for the number of candidates
k = g_candidates ^ 2; % Candidates per target
tau = 1; h_tau = 1; psi = 8; xi = 8;
N = 0; % Number of targets in all cameras
penalize_val = 1;
reward_val = -1;

gating_distance = 6.0; % This does actually correspond to 6 meters
score_threshold = 0.4;

initial_speed_x = 10.0;
initial_speed_y = 0;
sampling_plane = 'ground'; % NOTE Must be either 'camera' or 'ground'
dx = 10; dy = 50; % Sampling in the camera space
g_dx = 0.05; g_dy = 0.02; % Sampling in the

clustering = 'single';
comfort_distance = 1.5; % Anyone closer than this meters is in talking range?
FW_max_iterations = 5000;  % max number of iteration, I found out the opts.TOL (tolerance?) is more important in the convergence in my problem
FW_duality_gap = 2; % This sets the duality gap and you can change it if you want to change the tolerance for FW convergence
FW_eps = 1e-12; % (eps in matlab = 1e-16) not sure why this is needed
lambda = 0.5; % variable for the appearance cues
Alpha = [1.0 1.0]; % weight of the appearance constraint
Zeta = [1.0 1.0]; % weight of the motion constraint
a_sigma{1} = [1 ^ 2 0.0; 0.0 1 ^ 2]; a_sigma{2} = [1 ^ 2 0.0; 0.0 1 ^ 2]; a_sigma{3} = [1 ^ 2 0.0; 0.0 1 ^ 2];
m_sigma{1} = [0.5 0; 0 0.5]; m_sigma{2} = [0.5 0; 0 0.5]; m_sigma{3} = [0.5 0; 0 0.5];
G_sigma =  2 * (2 ^ 2);

h_group_distance = 0.5; % Estimate for the distance of groups made by homographies
h_FW_max_iterations = 5000;
h_FW_duality_gap = 2;
h_FW_eps = 1e-12;
h_lambda = 0.5;
h_Alpha = [1.0 1.0];
h_Zeta = [1.0 1.0];
h_a_sigma{1} = [1 ^ 2 0.0; 0.0 1 ^ 2]; h_a_sigma{2} = [1 ^ 2 0.0; 0.0 1 ^ 2]; h_a_sigma{3} = [1 ^ 2 0.0; 0.0 1 ^ 2];
h_m_sigma{1} = [0.5 0; 0 0.5]; h_m_sigma{2} = [0.5 0; 0 0.5]; h_m_sigma{3} = [0.5 0; 0 0.5];
h_G_sigma =  2 * (2 ^ 2);

appearance_method = 'naive'; % NOTE Must be either 'naive' or 'fourier'
intercam_method = 'frank-wolfe'; % NOTE Must be either 'lapjv' or 'frank-wolfe'
homog_solver = 'svd'; % Method to compute homographies, NOTE must be either 'svd' or 'ransac'
filter = 'none'; % NOTE must be either 'recursive' or 'non-recursive' or 'none' (kalman in the future?)
weights = cell(length(cameras),1); weights_spatial = cell(length(cameras),1);
%%=========================================================
debug_test_frames = 1; % DEBUG test these frames
homo_toggle = 1; % DEBUG Set this to 1 if you wish to only run with homography correction!
show_ground_truth = true;
show_candidates = true;
show_predicted_bbs = false;
draw_regions = true;
homocorrec_debug = 'no_debug';
candidates_frame = 2;
debug_gnd_truth_frames = 3;
nohomocorrec_tracklets = {};
%%=========================================================
% Frames at which detections are available/allowed
detection_frames = [1; 4; 9]; % These are used to updated targets/number of targets at these frames
%%=========================================================
fprintf('Starting tracking loop:\n');
for update_homo = homo_toggle:1 % DEBUG merely for debug, would never use this is "production"
    fprintf(['Updating homography set to ' num2str(update_homo) '\n']);
    all_candidates = cell(num_frames,1); % Store all candidates (in gnd plane)
    ground_plane_regions_adjusted = cell(length(cameras),1);
    valid_matchings = cell(length(cameras),1);
    groups = {}; adjusted_positions = cell(length(cameras),1); tracklets = {};

    for f = 1:(num_frames - 1)
        if f == debug_test_frames+1
            break; % DEBUG break cycle
        end
        fprintf(['  Frame ' num2str(f) ':\n \t 1.Getting images...\n']);
        next_images = cell(length(cameras),1); images = cell(length(cameras),1);
        for i = 1:length(cameras)
            try
                next_images{i} = imread(cameraListImages{i}{start_frames(i)+(f+1)});
                images{i} = imread(cameraListImages{i}{start_frames(i)+(f)});
            catch ME
                fprintf('Could not find dataset. If stored in a HDD, make sure it is mounted.\n');
            end
        end
        %---------------------------------------------------------------------------
        % TODO Detection frames should be used here
        fprintf('\t 2.Getting targets from frames in detection frames...\n');
        if ismember(f,detection_frames)
            targs = cell(length(cameras),1); % Targets from all cameras
            for id = 1:length(cameras)
                targs{id} = gnd_detections{id}{start_frames(id) + f};
            end
            targs = cell2mat(targs); % For some reason this work on one computer and on the other one does not
            % NOTE If the targets are empty on the first frame, can't track
            if isempty(targs)
                error('Cannot possibly track if there are no detections on the first given frame');
            end
        end

        targs_percam = (accumarray(targs(:,1),(1:size(targs,1)).',[],@(x){targs(x,:)},{}));
        N = size(targs,1);
        n = cell(length(cameras),1);
        for i=1:length(cameras)
            n{i} = size(targs_percam{i},1);
        end
        cands = cell(N,1); cands_percam = cell(length(cameras),1); % Candidates from both cameras for each target, always empty each frame
        %---------------------------------------------------------------------------
        % Sample around the targets. We then use these candidates on the next frame
        fprintf('\t 3.Sampling candidates...\n');
        for t = 1:size(targs,1)
            t_pos = targs(t,8:9);
            t_rect = targs(t,4:7);
            cands{t} = zeros(k,6);
            counter = 1;
            for gridx = -floor(g_candidates/2):floor(g_candidates/2)
              for gridy = -floor(g_candidates/2):floor(g_candidates/2)
                if strcmp(sampling_plane,'camera')
                    sampling_dx = t_rect(3)/dx;
                    sampling_dy = t_rect(4)/dy;
                    cx = t_rect(1) + gridx * sampling_dx;
                    cy = t_rect(2) + gridy * sampling_dy;
                    cands{t}(counter,:) = [cx cy t_rect(3:4) transpose(H(homographies{targs(t,1)}, [cx+t_rect(3)/2; cy+t_rect(4)]))];
                elseif strcmp(sampling_plane,'ground')
                    sampling_dx = g_dx; % TODO Bernardino proposed a way to actually get bb width from gnd plane
                    sampling_dy = g_dy; % TODO but we would need some data we do not have :(
                    g_cx = t_pos(1) + gridx * sampling_dx;
                    g_cy = t_pos(2) + gridy * sampling_dy;
                    c_pos = transpose(H(invhomographies{targs(t,1)}, [g_cx; g_cy]));
                    cands{t}(counter,:) = [c_pos(1) c_pos(2) t_rect(3:4) g_cx g_cy];
                end
                counter = counter + 1;
              end
            end
            cands_percam{targs(t,1)}{end+1} = cands{t};
        end
        all_candidates{f} = cands_percam;
        %---------------------------------------------------------------------------
        % check for potential new targets to add from the detections
        fprintf('\t 4.Checking for potential new targets...\n');
        % Check if detections are available. If there are more detections than currently there are targets consider this
        if f ~= 1
            if size(gnd_detections{id}{start_frames(id) + f},1) > size(targs_percam{id},1)
                fprintf('\t \t More detections than targets! \n');
            end
        end
        %---------------------------------------------------------------------------
        % NOTE store the ones that are ambiguous for homography correction in targs_in_overlap (i.e gating part 1)
        fprintf('\t 5.Checking for targets in the overlapping regions...\n')
        targs_in_overlap = {}; motion_models_overlap = {};
        for t = 1:size(targs,1)
            if polyin([targs(t,8) targs(t,9)],overlap)
                targs_in_overlap{end+1} = targs(t,:);
                if f ~= 1
                    motion_models_overlap{end+1} = [targs(t,1) transpose(motion_models{t})];
                end
            end
        end
        if ~isempty(targs_in_overlap)
            if f ~= 1
                motion_models_overlap = cell2mat(motion_models_overlap');
                motion_models_overlap = (accumarray(motion_models_overlap(:,1),(1:size(motion_models_overlap,1)).',[],@(x){motion_models_overlap(x,:)},{}));
            end
            targs_in_overlap = cell2mat(targs_in_overlap');
            targs_in_overlap = (accumarray(targs_in_overlap(:,1),(1:size(targs_in_overlap,1)).',[],@(x){targs_in_overlap(x,:)},{}));
            n_o = cell(length(cameras),1);
            for i = 1:length(cameras)
                n_o{i} = size(targs_in_overlap{i},1);
            end
            N_o = sum(n_o);
        end
        targs_o = cell2mat(targs_in_overlap);
    end
end

% NOTE Plot final tracks for debug mostly
%plot_output(all_candidates, ground_plane_regions, ground_plane_regions_adjusted, cameras, show_candidates, ...
%show_ground_truth, candidates_frame, draw_regions, tracklets, nohomocorrec_tracklets, gnd_detections, overlap_adjusted, ...
%num_frames,debug_gnd_truth_frames,start_frames);
