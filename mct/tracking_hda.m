setDetectionParams_hda_elevator;
setTrackerParams;

gnd_detections = load_data('hda');
% Load the images (needed for the appearance cues)
cameraListImages = cell(2,1); num_frames = [3779, 3720];
inplanes = cell(2,1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, num_frames(i), 0, 'hda');
    inplanes{i} = load(strcat(visibility_regions_directory,num2str(cameras{i}),'.mat')); inplanes{i} = inplanes{i}.t;
end
[homographies, invhomographies] = loadHomographies(homography_directory,'hda');
ground_plane_regions = computeGroundPlaneRegions(outlier_removal_regions, homographies, length(cameras), 'hda');
colors = {'Red','Blue','Black'};
