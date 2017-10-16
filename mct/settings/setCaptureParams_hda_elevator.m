global cameras
cameras = {57, 58};

global image_directory
image_directory = {'~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/cam'};

global crops_directories
crops_directories = {'~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/filtered_crops/'};

global homography_directory
homography_directory = '~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/homographies';

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
