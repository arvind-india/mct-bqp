%------------------------------------------------------
global cameras
cameras = {19, 40};

global hdaRootDirectory
hdaRootDirectory = '~/HDA_Dataset_V1.3';

global image_directories
image_directories = {'~/HDA_Dataset_V1.3/images/cam19/', '~/HDA_Dataset_V1.3/images/cam40/'};

global crops_directories
crops_directories = {'~/mct-bqp/hda_data/filtered_crops/'};

global homography_directory
homography_directory = '~/mct-bqp/hda_data/homographies';

global fps
fps = 5;

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

global pedestrian_detector
pedestrian_detector = 'AcfInria';

global crops
crops = {strcat(crops_directories, pedestrian_detector, '/allF_cam', int2str(cameras{1}), '.txt'), strcat(crops_directories, pedestrian_detector, '/allF_cam', int2str(cameras{2}), '.txt')};

global ground_truth
ground_truth = {strcat('~/mct-bqp/hda_data/ground_truth/', pedestrian_detector, '/allG_cam', int2str(cameras{1}), '.txt'), strcat('~/mct-bqp/hda_data/ground_truth/', pedestrian_detector, '/allG_cam', int2str(cameras{2}), '.txt')};

global visibility_regions_directory
visibility_regions_directory = '~/mct-bqp/hda_data/regions/visibility_points_image_';

global floor_image
floor_image = '~/mct-bqp/hda_data/regions/8th_floor_ground_plane_reference_frame_map.fig';
