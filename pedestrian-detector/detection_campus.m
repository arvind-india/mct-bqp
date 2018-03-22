addpath('/home/pedro/rcnn');
set(0,'DefaultFigureVisible','on');
% Set params of the detection
setDetectionParams_campus2;

cameraListImages = cell(2,1);
inplanes = cell(2,1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, 0, i, 'campus2');
    inplanes{i} = dlmread(strcat(regions_folder, cameras{i}, '.txt'));
end
sample_size = 7; % -1 means all
detections = CNNdetect(cameraListImages, sample_size, cameras, cpu_results, gpu_results, ...
 LDCF_cascThr, LDCF_cascCal, LDCF_rescale, use_GPU, score_threshold, NMS_maxoverlap);

% Parse for systematic error detections and remove all empty cells
detections = filterCNN(detections, inplanes);
for i=1:length(cameras)
    detections{i} = detections{i}(~cellfun('isempty',detections{i}));
end

start_frame = 65;
for i=1:length(cameras)
    figure;
    for s = start_frame:4:(sample_size+start_frame)
        waitforbuttonpress;
        clf;
        if i == 1
            plotDebugBoundingBoxes(cameraListImages,detections,s + offset_frames,'campus_2',i);
        else
            plotDebugBoundingBoxes(cameraListImages,detections,s,'campus_2',i);
        end
    end
end

% Load homographies, these homographies where obtained via the use of cp2tform
[homographies, invhomographies] = loadHomographies(homography_directory,'campus_2', cameras); % Defined in global variables

%%==============================================================
ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'campus_2');
clf;
figure; hold on;
colors = {'Red','Blue','Black'};
for i=1:length(cameras)
    drawPoly(ground_plane_regions{i},colors{i},0.5,false);
end
[overlap, ~, ~] = computeOverlap(ground_plane_regions);
drawPoly(overlap,colors{3},1.0,false);
%%===============================================================
% Convert all cam plane regions and store them if they don't exist already (this is the input to the tracker)
for i=1:length(cameras)
    fileID = fopen(['~/mct-bqp/campus2_data/detections-gndplane/', cameras{i} '.txt'],'w');
    for j=1:length(detections{i})
        %title(['Camera: ' cameras{i} ' Frame: ' j]);
        gnd_detections{i}{j} = zeros(size(detections{i}{j},1), 9);
        for d=1:size(detections{i}{j},1)
            t = H_alt(homographies{i}, [detections{i}{j}(d,3)+detections{i}{j}(d,5)/2 detections{i}{j}(d,4)+detections{i}{j}(d,6)]); % Get cam plane coordinates
            gnd_detections{i}{j}(d,1) = i;
            gnd_detections{i}{j}(d,2:7) = detections{i}{j}(d,1:6);

            % NOTE this converts from detection indexes to per cam post filtering indexes
            gnd_detections{i}{j}(d,3) = d;

            gnd_detections{i}{j}(d,8:9) = t; % x and y are now in the ground plane
            %if i == 1
            %    plot(t(1),t(2),'r+');
            %end
            %if i == 2
            %    plot(t(1),t(2),'b+');
            %end
            line = gnd_detections{i}{j}(d,:);
            formatSpec = '%d,%d,%d,%4.5f,%4.5f,%4.5f,%4.5f,%4.5f,%4.5f\n';
            fprintf(fileID,formatSpec,line(1),line(2),line(3),line(4),line(5),line(6),line(7),line(8),line(9));
        end
        %pause(0.1);
    end
end
