%------------------------------------------------------
global cameras
cameras = {57, 58};

global hdaRootDirectory
hdaRootDirectory = '/media/pedro/mct1/HDA_Dataset_V1.3';

global image_directories
image_directories = {'/media/pedro/mct1/HDA_Dataset_V1.3/images/cam57/', '/media/pedro/mct1/HDA_Dataset_V1.3/images/cam58/'};

global crops_directories
crops_directories = {'~/mct-bqp/hda_data/filtered_crops/'};

global homography_directory
homography_directory = '~/mct-bqp/hda_data/homographies';

global fps
fps = 2;

global dt
dt = 1/fps;

global resolutions
resolutions = [1280 800; 1280 800];

global durations
% 57 then 58
durations = [3780; 3721]; % Number of frames/fps

global offset
% Offset camera 58 to the camera 57 (58 is 64s later)
offset = 64; % in seconds

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
floor_image = '~/mct-bqp/hda_data/regions/7th_floor_ground_plane_reference_frame_map.fig';

global elevator_patio
elevator_patio = [198,167,100,100];
