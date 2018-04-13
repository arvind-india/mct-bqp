set(0,'DefaultFigureVisible','on');
setDetectionParams_ucla;
setTrackerParams;

gnd_detections = load_data('ucla', cameras); % Load the images (needed for the appearance cues)
cameraListImages = cell(2,1); inplanes = cell(2,1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, durations(i), 0, 'ucla');
    %inplanes{i} = load(strcat(visibility_regions_directory,num2str(cameras{i}),'.mat')); inplanes{i} = inplanes{i}.t;
end
[homographies, invhomographies] = loadHomographies(homography_directory,'ucla', cameras);
%ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'hda');
%[overlap, ~, ~] = computeOverlap(ground_plane_regions);

% TODO Remove entries with the lost or occlusion bit set to 1
for i=1:length(cameras)
    for j=1:size(gnd_detections{i},1)
        TF1 = gnd_detections{i}{j}(:,10)==1;
        TF2 = gnd_detections{i}{j}(:,11)==1;
        TFall = TF1 | TF2;
        gnd_detections{i}{j}(TFall,:) = [];
    end
end


figure; hold on;
for i=1:length(cameras)
    for j=2000:2500
        if i == 1
            scatter(gnd_detections{i}{j}(:,8),gnd_detections{i}{j}(:,9),'r');
        end
        if i == 2
            scatter(gnd_detections{i}{j}(:,8),gnd_detections{i}{j}(:,9),'b');
        end
        if i == 3
            scatter(gnd_detections{i}{j}(:,8),gnd_detections{i}{j}(:,9),'g');
        end
    end
end
