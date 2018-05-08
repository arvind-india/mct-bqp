set(0,'DefaultFigureVisible','on');
setDetectionParams_ucla;
setTrackerParams;
subset = '';
gnd_detections = load_data('ucla', cameras,subset); % Load the images (needed for the appearance cues)
cameraListImages = cell(length(cameras),1); inplanes = cell(length(cameras),1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, durations(i), 0, 'ucla');
    inplanes{i} = dlmread(strcat(regions_folder, cameras{i}, '.txt'));
    % TODO These are already in the 1024 format
    inplanes{i}(:,1) = inplanes{i}(:,1);
    inplanes{i}(:,2) = inplanes{i}(:,2);
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
% DEBUG I use the left foot of a man mounting his bike from frames 1 as a reference point
% DEBUG I use the the tip of the second white mark of the parkinglot to the right of the white car as another ref point
figure; hold on;
colors = {'Red','Blue','Green','Black'};
img_point_view1 = [48, 232 ; 542, 252];
img_point_view2 = [832, 204 ; 948, 291];
img_point_view3 = [422, 560 ; 146, 405];

for i = 1:length(cameras)
    for j = 1:size(img_point_view1,1)
        pt = img_point_view1(j,:);
        new_pt = H_ucla(homographies{i},pt);
        scatter(new_pt(1),new_pt(2),8,'MarkerFaceColor',rgb(colors{i}),'MarkerEdgeColor',rgb(colors{i}));
    end
end


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
    drawPoly(ground_plane_regions{i},colors{i},0.5,false); % Draw regions
end

figure; hold on;
sample_start = 1000; sample_size = 1000;
for i=1:length(cameras)
    for j=sample_start:(sample_start+sample_size)
        scatter(gnd_detections{i}{j}(:,8),gnd_detections{i}{j}(:,9),8,'MarkerFaceColor',rgb(colors{i}),'MarkerEdgeColor',rgb(colors{i}));
    end
end
