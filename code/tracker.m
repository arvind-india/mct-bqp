% trackerParams;
addpath('/opt/ibm/ILOG/CPLEX_Studio1271/cplex/matlab/x86-64_linux'); % NOTE ILog CPLEX
sampling_plane = 'camera'; % NOTE Must be either 'camera' or 'ground'
homog_solver = 'svd'; % NOTE Method to compute homographies, must be either 'svd' or 'ransac'
filter = 'none'; % NOTE must be either 'recursive' or 'none'
clustering = 'single';

hdaRootDirectory = '/media/pedro/mct/HDA_Dataset_V1.3';
dataset = 'hda';
seq = 'hall';
image_directories = {strcat(hdaRootDirectory,'/images/images_19/'), strcat(hdaRootDirectory,'/images/images_40/')};
homography_directory = '~/mct-bqp/data/hda/homographies';
visibility_regions_directory = '~/mct-bqp/data/hda/regions/visibility_points_image_';
floor_image = '~/mct-bqp/data/hda/regions/8th_floor_ground_plane_reference_frame_map.fig';

fps = 5.0;
dt = 1.0/fps;
cameras = {19, 40};
resolutions = [640 480; 640 480];
durations = [9877; 9860]; % Number of frames/fps
offset = 14; % Offset between camera 19 and 40 is in seconds
offset_frames = offset * fps;
start = 6812;
start_frames = [start + offset_frames start]; % Frames to start collecting images
k = 9 ^ 2; % Candidates per target (we choose it as a perfect square always)
reward_val = -1; % reward_val for bounds
dx = 5; dy = 10; % Sampling in the camera space
comfort_distance = 1.5; % Anyone closer than this meters is in talking range?
FW_max_iterations = 5000;  % max number of iteration, I found out the opts.TOL (tolerance?) is more important in the convergence in my problem
FW_duality_gap = 2; % This sets the duality gap and you can change it if you want to change the tolerance for FW convergence
FW_eps = 1e-12; % (eps in matlab = 1e-16) not sure why this is needed
Alpha = [1.0 1.0]; % weight of the appearance constraint
Zeta = [1.0 1.0]; % weight of the motion constraint
m_sigma = {0.5 0.5}; % TODO non-circular gaussian for motion
G_sigma =  8;
min_delta = 0.05; % NOTE minimum delta for homography correction loop
lambda = 0.5; % variable for the appearance cues
a_sigma = 100;
prefilter_sigma = 10;

gnd_detections = loadDetections(dataset, cameras, seq); % Load the images (needed for the appearance cues)
cameraListImages = loadImages(image_directories, durations, 0, length(cameras), dataset);
[homographies, invhomographies] = loadHomographies(homography_directory, dataset, cameras);
[inplanes, ground_plane_regions, overlap] = loadPlanes(visibility_regions_directory, homographies, cameras, length(cameras), dataset);

% openfig(floor_image); hold on; % Draw ground plane regions
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

tracklets = {};
valid_matchings = cell(2,1);
adjusted_positions = cell(2,1);
ground_plane_regions_adjusted = cell(2,1);

for f = 1:(num_frames - 1)
    if f == debug_test_frames + 1
        break; % DEBUG break cycle and force stopping
    end
    %---------------------------------------------------------------------------
    fprintf(['  Frame ' num2str(f) ':\n \t 1.Getting images\n']);
    % Gets f and (f+1) frames for all cameras
    [next_images, images] = getImages(length(cameras), cameraListImages, f, start_frames);
    %---------------------------------------------------------------------------
    fprintf('\t 2.Getting targets from frames in detection frames\n');
    [targs, targs_percam] = getTargets(f, detection_frames, gnd_detections, length(cameras), start_frames);
    n1 = size(targs_percam{1},1); n2 = size(targs_percam{2},1);
    N = n1 + n2;
    %---------------------------------------------------------------------------
    % Sample around the targets. We then use these candidates on the next frame
    fprintf('\t 3.Sampling candidates in f+1 frames\n');
    [cands, cands_percam] = sampleCandidates(N, k, length(cameras), targs, homographies, invhomographies, sampling_plane, dx, dy);
    %---------------------------------------------------------------------------
    % NOTE store the ones that are ambiguous for homography correction in targs_in_overlap (i.e gating part 1)
    fprintf('\t 4.Checking for targets in the overlapping regions...\n')
    [targs_in_overlap, n_o, N_o] = getTargetsOverlap(targs, overlap, cameras);
    %---------------------------------------------------------------------------
    fprintf('\t 5.Cloning targets...\n')
    cands_clones = getClones(N,length(cameras), k, targs_percam, overlap);
    %---------------------------------------------------------------------------
    fprintf('\t 6.Computing appearance cues (KCF)...\n');
    [a, weights, c_a, c_ca] = appearance(k, targs_percam, cands_percam, cands_clones, images, next_images, a_sigma, prefilter_sigma, lambda, weights, length(cameras), filter, dx, dy);

    %---------------------------------------------------------------------------
    fprintf('\t 7.Computing motion cues...\n');
    [m, targs_speed] = motion(k, targs_speed, targs_percam, cands_percam, gnd_detections, m_sigma, f, cands_clones, start_frames, length(cameras), dt);

    %---------------------------------------------------------------------------
    fprintf('\t 8.Computing bounding cues (assuming n1=n2 for now)...\n');
    b = bounds(k, n1, cands_percam, cands_clones, ground_plane_regions, reward_val, length(cameras));

    %---------------------------------------------------------------------------
    fprintf('\t 9.Computing grouping cues (for temporal association only)...\n');
    [G, groups] = grouping(N, [n1, n2], k, groups, targs_percam, cands_percam, f, G_sigma, length(cameras), comfort_distance);

    %---------------------------------------------------------------------------
    % NOTE join all cues and solve FW
    fprintf('\t 10.FW optimization! »» \n');
    % Prepare inputs for Frank Wolfe (conditional gradient)
    [H_,F,Aeq,Beq,labels] = FW_preamble(N, k, a, m, G, b, Alpha, Zeta, n1, n2);

    % Solve the problem using Frank Wolfe
    [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(H_, F, Aeq, Beq, labels, FW_max_iterations, FW_duality_gap, FW_eps, N); % minx is the value we want

    %---------------------------------------------------------------------------
    fsfsfs;
    optimization_results = reshape(minx,k,[]);
    optimization_results_percam = cell(2,1);
    opt_results_o = reshape(minx_o,2,[]);

    % TODO fix this, should work for multiple cameras
    n1_o = size(targs_in_overlap{1},1);
    n2_o = size(targs_in_overlap{2},1);
    optimization_results_percam{1} = optimization_results(:,1:n1);
    optimization_results_percam{2} = optimization_results(:,n1:end);
    opt_results_percam_spatial{1} = opt_results_o(:,1:n1_o);
    opt_results_percam_spatial{2} = opt_results_o(:,n1_o+1:end);
    valid_matchings = getValidMatchings_fw(n_o,cameras,opt_results_percam_spatial,targs_in_overlap,valid_matchings);

    fprintf('\t Found optimal candidates for each target.\n');
    %---------------------------------------------------------------------------
    printf('\t Parsing for spatial association.\n');
    % Parse FW results for this

    fprintf('\t Parsing for spatial association.\n');
    % Parse FW results for this

    %---------------------------------------------------------------------------
    % NOTE homography Correction must be done separately, independently of how the target coupling is solved
    % Correct homographies and ALL detections using these homographies
    [homographies, invhomographies, adjusted_gnd_detections, adjusted_ground_plane_regions, adjusted_overlap] = homography_correction(matchings, inplanes, ground_plane_regions, homog_solver, num_cams);

    %---------------------------------------------------------------------------
    % NOTE update motion models using old targets and the chosen candidates
    % update
end
