%------------------------------------------------------
global cameras
cameras = {'view-GL1', 'view-GL2', 'view-GL5'};

global uclaRootDirectory
uclaRootDirectory = '/media/pedro/mct1/UCLA';

global image_directories
image_directories = {strcat(uclaRootDirectory,'/view-GL1/'), strcat(uclaRootDirectory,'/view-GL2/'), strcat(uclaRootDirectory,'/view-GL5/')};

global regions_folder
regions_folder = '~/mct-bqp/ucla_data/regions/';

global homography_directory
homography_directory = '~/mct-bqp/ucla_data/homographies/cam_param.mat';

global fps
fps = 30.0;

global dt
dt = 1/fps;

global resolutions
resolutions = [1920 1080; 1920 1080];

global durations
durations = [6475; 6475; 6473]; % Number of frames/fps (for some reason a cam lost 2 frames)

global offset
% Offset between camera 19 and 40 is:
offset = 0; % in seconds

global offset_frames
offset_frames = offset * fps;
