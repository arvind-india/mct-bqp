set(0,'DefaultFigureVisible','on');
setDetectionParams_hda_hall_3cams; setTrackerParams;

gnd_detections = load_data('hda', cameras); % Load the images (needed for the appearance cues)
cameraListImages = cell(length(cameras),1); inplanes = cell(length(cameras),1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, durations(i), 0, 'hda');
    inplanes{i} = load(strcat(visibility_regions_directory, num2str(cameras{i}),'.mat')); inplanes{i} = inplanes{i}.t;
end
[homographies, invhomographies] = loadHomographies(homography_directory,'hda', cameras);
ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'hda');
[overlap, ~, ~] = computeOverlap(ground_plane_regions);
%%=====================================================================
