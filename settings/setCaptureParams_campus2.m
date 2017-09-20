global cameras
cameras = {'alameda','central'};

global image_directories
image_directories = {'~/Campus_II/frames_alameda_noon_1_6_2017', '~/Campus_II/frames_central_noon_1_6_2017'};

global homography_directory
homography_directory = '~/Campus_II/homography_campus_II.mat'

global fps
fps = 9;

global resolutions
resolutions = [1024 768; 1024 768];

global durations
% Alameda then Central
durations = [15*60+2; 14*60+58];

global offset
% Offset camera Central to the camera Alameda (Central is 40s later)
offset = 40; % seconds
