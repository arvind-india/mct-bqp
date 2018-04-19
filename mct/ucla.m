set(0,'DefaultFigureVisible','on');
setDetectionParams_ucla;
setTrackerParams;

gnd_detections = load_data('ucla', cameras); % Load the images (needed for the appearance cues)
cameraListImages = cell(2,1); inplanes = cell(2,1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, durations(i), 0, 'ucla');
    inplanes{i} = dlmread(strcat(regions_folder, cameras{i}, '.txt'));
end
[homographies, invhomographies] = loadHomographies(homography_directory,'ucla', cameras);

ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'ucla');
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
colors = {'Red','Blue','Green','Black'};
figure; hold on;
for i=1:length(cameras)
    drawPoly(ground_plane_regions{i},colors{i},0.5,false); % Draw regions
end

figure; hold on;
for i=1:length(cameras)
    for j=2000:2500
        if i == 1
            scatter(gnd_detections{i}{j}(:,8),gnd_detections{i}{j}(:,9),8,'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red'));
        end
        if i == 2
            scatter(gnd_detections{i}{j}(:,8),gnd_detections{i}{j}(:,9),8,'MarkerFaceColor',rgb('Blue'),'MarkerEdgeColor',rgb('Blue'));
        end
        if i == 3
            scatter(gnd_detections{i}{j}(:,8),gnd_detections{i}{j}(:,9),8,'MarkerFaceColor',rgb('Green'),'MarkerEdgeColor',rgb('Green'));
        end
    end
end
