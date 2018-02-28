addpath('/home/pedro/rcnn');
set(0,'DefaultFigureVisible','on');
% Set params of the detection
setDetectionParams_hda_elevator;

cameraListImages = cell(2,1); num_frames = [3779, 3720];
inplanes = cell(2,1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, num_frames(i), 0, 'hda');
    inplanes{i} = load(strcat(visibility_regions_directory,num2str(cameras{i}),'.mat')); inplanes{i} = inplanes{i}.t;
end

detect_mode = 'GT'; % GT, CNN, ACF

if strcmp(detect_mode,'GT') == 1
    % TODO Get ground truth
    gt = cell(length(cameras),1);
    for c=1:length(cameras)
      nmVbb = [hdaRootDirectory '/hda_annotations/cam' int2str(cameras{c}) '.txt'];
      annotations = vbb('vbbLoad', nmVbb);
      gt{c} = annotations.objLists;
    end
    detections = gt;
elseif strcmp(detect_mode,'CNN') == 1
    % TODO Get DeepPed detections



elseif strcmp(detect_mode,'ACF') == 1
    % TODO Get ACF detections

end

[homographies, invhomographies] = loadHomographies(homography_directory,'hda');

ground_plane_regions = computeGroundPlaneRegions(outlier_removal_regions, homographies, length(cameras), 'hda');
openfig(floor_image); hold on;
colors = {'Red','Blue'};
for i=1:length(cameras)
    drawPoly(ground_plane_regions{i},colors{i},0.5,false); % Draw regions
end

%%===============================================================
% Convert all cam plane regions and store them if they don't exist already (this is the input to the tracker)
for i=1:length(cameras)
    fileID = fopen(['~/mct-bqp/hda_data/detections-gndplane/', cameras{i} '.txt'],'w');
    for j=1:length(detections{i})
        title(['Camera: ' cameras{i} ' Frame: ' j]);
        gnd_detections{i}{j} = zeros(size(detections{i}{j},1), 9);
        for d=1:size(detections{i}{j},1)
            t = H_alt(homographies{i}, [detections{i}{j}(d,3)+detections{i}{j}(d,5)/2 detections{i}{j}(d,4)+detections{i}{j}(d,6)]); % Get cam plane coordinates
            gnd_detections{i}{j}(d,1) = i;
            gnd_detections{i}{j}(d,2:7) = detections{i}{j}(d,1:6);

            % NOTE this converts from detection indexes to per cam post filtering indexes
            gnd_detections{i}{j}(d,3) = d;

            gnd_detections{i}{j}(d,8:9) = t; % x and y are now in the ground plane
            if i == 1
                plot(t(1),t(2),'r+');
            end
            if i == 2
                plot(t(1),t(2),'b+');
            end
            line = gnd_detections{i}{j}(d,:);
            formatSpec = '%d,%d,%d,%4.5f,%4.5f,%4.5f,%4.5f,%4.5f,%4.5f\n';
            fprintf(fileID,formatSpec,line(1),line(2),line(3),line(4),line(5),line(6),line(7),line(8),line(9));
        end
        %pause(0.1);
    end
end
