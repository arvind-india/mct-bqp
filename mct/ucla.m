set(0,'DefaultFigureVisible','on');
setDetectionParams_ucla;
setTrackerParams;
subset = '';
gnd_detections = load_data('ucla', cameras,subset); % Load the images (needed for the appearance cues)
cameraListImages = cell(length(cameras),1); inplanes = cell(length(cameras),1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, durations(i), 0, 'ucla');
    inplanes{i} = dlmread(strcat(regions_folder, cameras{i}, '.txt'));
    % TODO Make this prettier in the future
    inplanes{i}(:,1) = inplanes{i}(:,1) * (1024/1920);
    inplanes{i}(:,2) = inplanes{i}(:,2) * (576/1080);
end

[homographies, invhomographies] = loadHomographies(homography_directory,'ucla', cameras);

homoplanes = {};
for i=1:length(cameras) % For the cameras
    homoplanes{i} = zeros(size(inplanes{i},1),2);
    for p=1:size(inplanes{i},1)
        pts = inplanes{i}(p,:);
        new_pts = H_ucla(homographies{i},pts);
        homoplanes{i}(p,:) = new_pts;
    end
end
for i=1:length(cameras)
    ground_plane_regions{i} = [homoplanes{i};homoplanes{i}(1,:)];
    inplanes{i} = [inplanes{i};inplanes{i}(1,:)];
end

%ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'ucla');
%[overlap, ~, ~] = computeOverlap(ground_plane_regions);

% Debug points that should overlap from all views



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
    for j=1000:2000
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
