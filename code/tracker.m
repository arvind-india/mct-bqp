addpath('/opt/ibm/ILOG/CPLEX_Studio1271/cplex/matlab/x86-64_linux'); % NOTE ILog CPLEX
sampling_plane = 'camera'; % NOTE Must be either 'camera' or 'ground'
homog_solver = 'svd'; % NOTE Method to compute homographies, must be either 'svd' or 'ransac'
filter = 'recursive'; % NOTE must be either 'recursive' or 'none'
clustering = 'single';

% dataset = 'hda';
% cameras = {19, 40};
% durations = [9877; 9860]; % Number of frames/fps
% offset = 14; % Offset between camera 19 and 40 is in seconds
% rootDirectory = '/media/pedro/mct/HDA_Dataset_V1.3';
% image_directories = {strcat(rootDirectory,'/images/images_19/'), strcat(rootDirectory,'/images/images_40/')};
% homography_directory = '~/mct-bqp/data/hda/homographies';
% visibility_regions_directory = '~/mct-bqp/data/hda/regions/visibility_points_image_';
% floor_image = '~/mct-bqp/data/hda/regions/8th_floor_ground_plane_reference_frame_map.fig';
% fps = 5.0;
% dt = 1.0/fps;
% resolutions = [640 480; 640 480];
% offset_frames = offset * fps;
% start = 6812;
% start_frames = [start + offset_frames start]; % Frames to start collecting images
% k = 9 ^ 2; % Candidates per target (we choose it as a perfect square always)
% reward_val = -1; % reward_val for bounds
% dx = 5; dy = 10; % Sampling in the camera space
% comfort_distance = 1.5; % Anyone closer than this meters is in talking range?
% FW_max_iterations = 5000;  % max number of iteration, I found out the opts.TOL (tolerance?) is more important in the convergence in my problem
% FW_duality_gap = 0.001; % This sets the duality gap and you can change it if you want to change the tolerance for FW convergence
% FW_eps = 1e-12; % (eps in matlab = 1e-16) not sure why this is needed
% Alpha = [1.0 1.0]; % weight of the appearance constraint
% Mu = [1.0 1.0]; % weight of the motion constraint
% m_sigma = {0.5 0.5}; % TODO non-circular gaussian for motion
% G_sigma =  8;
% min_delta = 0.05; % NOTE minimum delta for homography correction loop
% lambda = 0.5; % variable for the appearance cues
% a_sigma = 100;
% prefilter_sigma = 10;
% num_frames = 7;
% detection_frames = [1 5];
% stop_frame = 2; % DEBUG Frame to stop at
% homo_correction = false; % NOTE Set to false if not homography_correction
% resampling = false;
% clustering = 'single';

rootDirectory = '/media/pedro/mct/UCLA/Parkinglot';
dataset = 'ucla';
image_directories = {strcat(rootDirectory,'/images/view-GL1/'), strcat(rootDirectory,'/images/view-GL2/')};
homography_directory = '~/mct-bqp/data/ucla/homographies';
visibility_regions_directory = '~/mct-bqp/data/ucla/regions/';
% floor_image = '~/mct-bqp/data/hda/regions/8th_floor_ground_plane_reference_frame_map.fig';
fps = 30.0;
dt = 1.0/fps;
cameras = {1, 2};
resolutions = [1024 576; 1024 576];
durations = [6475; 6475]; % Number of frames/fps (UCLA is calibrated :D)
offset = 0;
offset_frames = offset * fps;
start = 6136;
start_frames = [start + offset_frames start]; % Frames to start collecting images
k = 9 ^ 2; % Candidates per target (we choose it as a perfect square always)
reward_val = -1; % reward_val for bounds
dx = 4; dy = 10; % Sampling in the camera space
comfort_distance = 1.5; % Anyone closer than this meters is in talking range? (it depends on the dataset, because homographies can be very innacurate)
FW_max_iterations = 5000;  % max number of iteration, I found out the opts.TOL (tolerance?) is more important in the convergence in my problem
FW_duality_gap = 0.001; % This sets the duality gap and you can change it if you want to change the tolerance for FW convergence
FW_eps = 1e-12; % (eps in matlab = 1e-16) not sure why this is needed

Alpha = [1.0 1.0]; % weight of the appearance constraint
Mu = [1.0 1.0]; % weight of the motion constraint
Beta = 0.0;
Gamma = 0.0; % weight of grouping constraint

m_sigma = {3 3}; % TODO non-circular gaussian for motion
G_sigma =  8;
min_delta = 0.05; % NOTE minimum delta for homography correction loop
lambda = [20 10]; % variable for the appearance cues
a_sigma = [60 70];
prefilter_sigma = 2;
num_frames = 7;
detection_frames = [1 3];
stop_frame = 2; % DEBUG Frame to stop at
homo_correction = false; % NOTE Set to false if not homography_correction
resampling = false;
clustering = 'single'; % NOTE Type of hierarchical clustering
cluster_update_freq = 4; % NOTE Every cluster_update_freq frames update clusters
people = [6 11]; % NOTE Only makes sense in UCLA

gnd_detections = loadDetections(dataset, cameras); % Load the images (needed for the appearance cues)
cameraListImages = loadImages(image_directories, durations, 0, length(cameras), dataset);
[homographies, invhomographies] = loadHomographies(homography_directory, dataset, cameras);
[inplanes, ground_plane_regions, overlap] = loadPlanes(visibility_regions_directory, homographies, cameras, length(cameras), dataset);

% openfig(floor_image); hold on; % DEBUG Draw ground plane regions
% colors = {'Red','Blue','Black'};
% for i=1:length(cameras)
%     drawPoly(ground_plane_regions{i},colors{i},0.5,false); % Draw regions
% end
% drawPoly(overlap,colors{end},1.0,false);

%%=========================================================
fprintf('Starting tracking loop:\n');
targs_speed = {};
weights = {};
groups = {};
prev_alphaf = -1;
tracks = cell(length(cameras),1);
for c = 1:length(cameras)
    tracks{c} = cell(max(people) + 1,1); % NOTE +1 cuz of matlab *sigh*
end

for f = 1:(num_frames - 1)
    if f == stop_frame
        break; % DEBUG break cycle and force stopping for debug
    end
    %---------------------------------------------------------------------------
    fprintf(['  Frame ' num2str(f) ':\n \t 1.Getting images ']);
    tic
    % Gets f and (f+1) frames for all cameras
    [next_images, images] = getImages(length(cameras), cameraListImages, f, start_frames);
    tval = toc; fprintf(['(' num2str(tval) ' s) \n']);
    %---------------------------------------------------------------------------
    fprintf('\t 2.Getting targets from frames in detection frames ');
    tic
    [targs, targs_percam] = getTargets(f, detection_frames, gnd_detections, length(cameras), start_frames);
    [targs, targs_percam] = getDebugTargets(targs, targs_percam, people);
    % TODO This is hardcoded for 2 cameras, fix this
    n1 = size(targs_percam{1},1); n2 = size(targs_percam{2},1);
    N = n1 + n2;
    tval = toc; fprintf(['(' num2str(tval) ' s) \n']);
    %---------------------------------------------------------------------------
    % Sample around the targets. We then use these candidates on the next frame
    fprintf('\t 3.Sampling candidates in f+1 frames ');
    tic
    [cands, cands_percam] = sampleCandidates(N, k, length(cameras), targs, homographies, invhomographies, sampling_plane, dx, dy, dataset);
    tval = toc; fprintf(['(' num2str(tval) ' s) \n']);
    %---------------------------------------------------------------------------
    % NOTE store the ones that are ambiguous for homography correction in targs_in_overlap (i.e gating part 1)
    fprintf('\t 4.Checking for targets in the overlapping regions ')
    tic
    [targs_in_overlap, n_o, N_o] = getTargetsOverlap(targs, overlap, cameras);
    tval = toc; fprintf(['(' num2str(tval) ' s) \n']);
    %---------------------------------------------------------------------------
    fprintf('\t 5.Getting clone targets ')
    tic
    cands_clones = getClones(N,length(cameras), k, targs_percam, overlap);
    tval = toc; fprintf(['(' num2str(tval) ' s) \n']);
    %---------------------------------------------------------------------------
    fprintf('\t 6.Computing appearance cues (KCF) ');
    tic
    [a, weights, c_a, c_ca, alphaf] = appearance(k, targs_percam, cands_percam, cands_clones, images, next_images, a_sigma, prefilter_sigma, lambda, weights, length(cameras), filter, dx, dy, dataset, prev_alphaf);
    prev_alphaf = alphaf;
    tval = toc; fprintf(['(' num2str(tval) ' s) \n']);
    %---------------------------------------------------------------------------
    fprintf('\t 7.Computing motion cues '); % NOTE Initializing of motion models is done here too
    tic
    [m, targs_speed] = motion(k, targs_speed, targs_percam, cands_percam, gnd_detections, m_sigma, f, cands_clones, start_frames, length(cameras), dt, detection_frames);
    correct_targs_speed = targs_speed;
    tval = toc; fprintf(['(' num2str(tval) ' s) \n']);
    %---------------------------------------------------------------------------
    fprintf('\t 8.Computing bounding cues ');
    tic
    b = bounds(k, n1, cands_percam, cands_clones, ground_plane_regions, reward_val, length(cameras));
    tval = toc; fprintf(['(' num2str(tval) ' s) \n']);

    %---------------------------------------------------------------------------
    if resampling == true
        fprintf('\t Re-sampling candidates.\n');
    end
    %---------------------------------------------------------------------------
    fprintf('\t 9.Computing grouping cues (for temporal association only) '); % NOTE Initializing of motion models is done here too
    tic
    [G, groups, dendrograms] = grouping(N, [n1, n2], k, groups, targs_percam, cands_percam, f, G_sigma, length(cameras), comfort_distance, dataset, cluster_update_freq, clustering, Gamma);
    G = zeros(size(G,1), size(G,2));
    tval = toc; fprintf(['(' num2str(tval) ' s) \n']);

    %---------------------------------------------------------------------------
    % NOTE join all cues and solve FW
    fprintf('\t 10.Frank-Wolfe optimization \n');
    tic
    % Prepare inputs for Frank Wolfe (conditional gradient)
    % TODO Remake this so n1 and n2 are not needed
    [H_,F,Aeq,Beq,labels] = FW_preamble(N*2, k, a, m, G, b, Alpha, Mu, n1, n2);

    % Solve the problem using Frank Wolfe
    [x,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(H_, F, Aeq, Beq, labels, FW_max_iterations, FW_duality_gap, FW_eps, N); % minx is the value we want

    %---------------------------------------------------------------------------
    % Reshape x so it's easier to parse
    x = reshapeX(x, cameras, k, targs_percam);

    tval = toc; fprintf(['\t (' num2str(tval) ' s) \n']);
    %---------------------------------------------------------------------------
    fprintf('\t Parsing for temporal association.\n');
    % Parse FW results for this
    [best_temporal, best_temporal_candidates] = parseTemporal(x, k, cands_percam, length(cameras));

    fprintf('\t Parsing for spatial association (get matchings).\n');
    % Parse FW results for this
    [best_spatial, best_spatial_candidates, spatial_matchings] = parseSpatial(x, k, cands_clones, length(cameras));

    %---------------------------------------------------------------------------
    % Correct homographies and ALL detections using these homographies (homography Correction can be done separately, independently of how the target coupling is solved)
    if homo_correction == true
        [homographies, invhomographies, gnd_detections, ground_plane_regions, overlap] = homography_correction(spatial_matchings, inplanes, ground_plane_regions, homog_solver, length(cameras));
    end

    %---------------------------------------------------------------------------
    % DEBUG Plot data and cues
    plotData(targs_percam, cands_percam, best_temporal_candidates, best_spatial_candidates, next_images, images, length(cameras), dataset, false, ground_plane_regions); % Plots targets and candidates (camera and ground plane)
    plotCues(a, m, b, dendrograms, next_images, cands_percam, k, length(cameras), comfort_distance); % Plots appearance, motion, bounds and grouping (dendrograms) data

    %---------------------------------------------------------------------------
    % NOTE update motion models using old targets and the chosen candidates
    [targs_speed, tracks] = update(targs_speed, targs_percam, tracks, best_temporal_candidates, length(cameras), dt, dataset);
end

%---------------------------------------------------------------------------
% NOTE evaluate tracking target vs camera detection. Evaluation of ground plane detections? (distance between targets)
correct_tracks = evaluate(f, gnd_detections, tracks, people, start_frames, length(cameras), overlap);
