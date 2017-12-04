global cameras
cameras = {57, 58};

global image_directory
image_directory = {'~/mct-bqp/hda_data/images/cam'};

global crops_directories
crops_directories = {'~/mct-bqp/hda_data/filtered_crops/'};

global homography_directory
homography_directory = '~/mct-bqp/hda_data/homographies';

global fps
fps = 2;

global resolutions
resolutions = [1280 800; 1280 800];

global durations
% 57 then 58
durations = [3780/fps; 3721/fps]; % Number of frames/fps

global offset
% Offset camera 58 to the camera 57 (58 is 40s later)
offset = 64/fps; % seconds
