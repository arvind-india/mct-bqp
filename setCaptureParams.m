global cameras
cameras = {'alameda','central'};

global image_directories
image_directories = {'~/Campus_II/frames_alameda_noon_1_6_2017', '~/Campus_II/frames_central_noon_1_6_2017'};

global fps
fps = 9;

global resolutions
resolutions = [1024 768; 1024 768];

global durations
%alameda then central
durations = [15*60+2; 14*60+58];

global offset
%Offset camera central to the camera alameda
offset = 40; %39-40
