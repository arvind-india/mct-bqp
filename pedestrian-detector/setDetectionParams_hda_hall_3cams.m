%------------------------------------------------------
global cameras
cameras = {18, 19, 40};

global hdaRootDirectory
%hdaRootDirectory = '/media/pedro/mct1/HDA_Dataset_V1.3';
hdaRootDirectory = '~/HDA_Dataset_V1.3';
global image_directories
image_directories = {strcat(hdaRootDirectory,'/images/cam18/'),strcat(hdaRootDirectory,'/images/cam19/'), strcat(hdaRootDirectory,'/images/cam40/')};

global crops_directories
crops_directories = {'~/mct-bqp/hda_data/filtered_crops/'};

global homography_directory
homography_directory = '~/mct-bqp/hda_data/homographies';

global fps
fps = 5;

global dt
dt = 1/fps;

global resolutions
resolutions = [640 480; 640 480; 640 480];

global durations
durations = [9885; 9877; 9860]; % Number of frames/fps

global offset
% Offset between camera 18 and 19 and 40 is:
offset = [0, 2, 14]; % in seconds

global offset_frames
offset_frames = offset * fps;
%------------------------------------------------------------

global pedestrian_detector
pedestrian_detector = 'AcfInria';

global crops
crops = {strcat(crops_directories, pedestrian_detector, '/allF_cam', int2str(cameras{1}), '.txt'), strcat(crops_directories, pedestrian_detector, '/allF_cam', int2str(cameras{2}), '.txt')};

global ground_truth
ground_truth = {strcat('~/mct-bqp/hda_data/ground_truth/', pedestrian_detector, '/allG_cam', int2str(cameras{1}), '.txt'), strcat('~/mct-bqp/hda_data/ground_truth/', pedestrian_detector, '/allG_cam', int2str(cameras{2}), '.txt'), strcat('~/mct-bqp/hda_data/ground_truth/', pedestrian_detector, '/allG_cam', int2str(cameras{3}), '.txt')};

global visibility_regions_directory
visibility_regions_directory = '~/mct-bqp/hda_data/regions/visibility_points_image_';

global floor_image
floor_image = '~/mct-bqp/hda_data/regions/8th_floor_ground_plane_reference_frame_map.fig';

%-------------------------------------------------------------------------------
% The parameters 'nPerOct', 'nOctUp', 'nApprox', 'lambdas', 'pad', 'minDs'
% modify the channel feature pyramid created (see help of chnsPyramid.m for
% more details) and primarily control the scales used.

%  .nPerOct    - [] number of scales per octave
%  .nOctUp     - [] number of upsampled octaves to compute
%  .nApprox    - [] number of approx. scales to use
%  .lambdas    - [] coefficients for power law scaling (see BMVC10)
%  .pad        - [] amount to pad channels (along T/B and L/R)
%  .minDs      - [] minimum image size for channel computation

%-------------------------------------------------------------------------------
% The parameters 'pNms', 'stride', 'cascThr' and 'cascCal' modify the detector behavior
% (see help of acfTrain.m for more details). Finally, 'rescale' can be
% used to rescale the trained detector (this change is irreversible).

% Typically, set 'cascThr' to -1 and adjust 'cascCal' until the desired recall is reached
% (setting 'cascCal' shifts the final scores output by the detector by the given amount)

% .cascThr    - [] constant cascade threshold (affects speed/accuracy)
global LDCF_cascThr
LDCF_cascThr = [-1 -1];

% .cascCal    - [] cascade calibration (affects speed/accuracy)
global LDCF_cascCal
% 0.020 0.025 0.030 0.035
LDCF_cascCal = [0.029 0.028];

% .rescale    - [] rescale entire detector by given ratio, irreversible
global LDCF_rescale
LDCF_rescale = [1.0 1.0];

% .pNms       - [] params for non-maximal suppression (see bbNms.m)
global pNMS_overlap
pNMS_overlap = [1.0 1.0];

% .stride     - [] spatial stride between detection windows
global LDCF_stride
LDCF_stride = [1 1];

%-------------------------------------------------------------------------------
% The following are used to filter out the CNN detections
global score_threshold
% 0.25 0.5 0.75 1.0
score_threshold = [0.8 0.6];

global NMS_maxoverlap
% 0.25 0.5 0.75 0.9
NMS_maxoverlap = [0.9 1.0];

%-------------------------------------------------------------------------------
global order
order = 'cardinal';

global cpu_results
cpu_results = '/home/pedro/mct-bqp/hda_data/detections-camplane/CPU/dets_';

global gpu_results
gpu_results = '/home/pedro/mct-bqp/hda_data/detections-camplane/GPU/dets_';

global use_GPU
use_GPU = [1 1];
