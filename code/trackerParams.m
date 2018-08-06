% Change folders and options
global cplex_folder
cplex_folder = '/opt/ibm/ILOG/CPLEX_Studio1271/cplex/matlab/x86-64_linux';
addpath(cplex_folder);

sampling_plane = 'camera'; % NOTE Must be either 'camera' or 'ground'

clustering = 'single';

homog_solver = 'svd'; % Method to compute homographies, NOTE must be either 'svd' or 'ransac'
filter = 'none'; % NOTE must be either 'recursive' or 'non-recursive' or 'none' (kalman in the future?)

homocorrec_debug = 'no_debug';
% ======================================================
% Change according to the dataset

global cameras
cameras = {19, 40};

global hdaRootDirectory
hdaRootDirectory = '/media/pedro/mct/HDA_Dataset_V1.3';

global image_directories
image_directories = {strcat(hdaRootDirectory,'/images/images_19/'), strcat(hdaRootDirectory,'/images/images_40/')};

global homography_directory
homography_directory = '~/mct-bqp/data/hda/homographies';

global fps
fps = 5.0;

global dt
dt = 1/fps;

global resolutions
resolutions = [640 480; 640 480];

global durations
durations = [9877; 9860]; % Number of frames/fps

global offset
% Offset between camera 19 and 40 is:
offset = 14; % in seconds

global offset_frames
offset_frames = offset * fps;
%------------------------------------------------------------
%global ground_truth
%ground_truth = {strcat('~/mct-bqp/hda_data/ground_truth/', pedestrian_detector, '/allG_cam', int2str(cameras{1}), '.txt'), strcat('~/mct-bqp/hda_data/ground_truth/', pedestrian_detector, '/allG_cam', int2str(cameras{2}), '.txt')};

global visibility_regions_directory
visibility_regions_directory = '~/mct-bqp/data/hda/regions/visibility_points_image_';

global floor_image
floor_image = '~/mct-bqp/data/hda/regions/8th_floor_ground_plane_reference_frame_map.fig';

%-------------------------------------------------------------------------------
global order
order = 'cardinal';





% ======================================================
% Change tracker math params
num_frames = 7; % Number of frames
start = 6812;
start_frames = [start + offset_frames start]; % Frames to start collecting images

k = 9 ^ 2; % Candidates per target
tau = 1; h_tau = 1; psi = 8; xi = 8;
N = 0; % Number of targets in all cameras
penalize_val = 1;
reward_val = -1;

gating_distance = 6.0; % This does actually correspond to 6 meters
score_threshold = 0.4;

initial_speed_x = 10.0;
initial_speed_y = 0;

dx = 5; dy = 10; % Sampling in the camera space
g_dx = 0.05; g_dy = 0.02; % Sampling in the


comfort_distance = 1.5; % Anyone closer than this meters is in talking range?
FW_max_iterations = 5000;  % max number of iteration, I found out the opts.TOL (tolerance?) is more important in the convergence in my problem
FW_duality_gap = 2; % This sets the duality gap and you can change it if you want to change the tolerance for FW convergence
FW_eps = 1e-12; % (eps in matlab = 1e-16) not sure why this is needed
lambda = 0.5; % variable for the appearance cues
Alpha = [1.0 1.0]; % weight of the appearance constraint
Zeta = [1.0 1.0]; % weight of the motion constraint
a_sigma{1} = [1 ^ 2 0.0; 0.0 1 ^ 2]; a_sigma{2} = [1 ^ 2 0.0; 0.0 1 ^ 2];
m_sigma{1} = [0.5 0; 0 0.5]; m_sigma{2} = [0.5 0; 0 0.5];
G_sigma =  2 * (2 ^ 2);

h_group_distance = 0.5; % Estimate for the distance of groups made by homographies
h_FW_max_iterations = 5000;
h_FW_duality_gap = 2;
h_FW_eps = 1e-12;
h_lambda = 0.5;
h_Alpha = [1.0 1.0];
h_Zeta = [1.0 1.0];
h_a_sigma{1} = [1 ^ 2 0.0; 0.0 1 ^ 2]; h_a_sigma{2} = [1 ^ 2 0.0; 0.0 1 ^ 2];
h_m_sigma{1} = [0.5 0; 0 0.5]; h_m_sigma{2} = [0.5 0; 0 0.5];
h_G_sigma =  2 * (2 ^ 2);

min_delta = 0.05; % NOTE minimum delta for homography correction loop


weights = cell(2,1); weights_spatial = cell(2,1);
%%=========================================================
debug_test_frames = 1; % DEBUG test these frames
show_ground_truth = true;
show_candidates = true;
show_predicted_bbs = false;
draw_regions = true;
candidates_frame = 2;
debug_gnd_truth_frames = 3;
nohomocorrec_tracklets = {};
accumulated_Zs = cell(length(cameras),1);
%%=========================================================
% Frames at which detections are available/allowed
detection_frames = [1; 4; 9]; % These are used to updated targets/number of targets at these frames
