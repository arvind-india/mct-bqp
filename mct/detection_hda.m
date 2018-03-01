addpath('/home/pedro/rcnn');
set(0,'DefaultFigureVisible','on');
% Set params of the detection
setDetectionParams_hda_hall;

cameraListImages = cell(2,1);
inplanes = cell(2,1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, durations(i), 0, 'hda');
    inplanes{i} = load(strcat(visibility_regions_directory,num2str(cameras{i}),'.mat')); inplanes{i} = inplanes{i}.t;
end

detect_mode = 'GT'; % GT, CNN, ACF
detections = cell(2,1); detections_inclocll = cell(2,1); % NOTE We have all occluded detections included on this second variable
if strcmp(detect_mode,'GT') == 1
    % TODO Get ground truth
    gt = cell(length(cameras),1);
    for c=1:length(cameras)
      nmVbb = [hdaRootDirectory '/hda_annotations/cam' int2str(cameras{c}) '.txt'];
      annotations = vbb('vbbLoad', nmVbb);
      gt{c} = annotations.objLists;
      for t = 1:length(gt{c})
          detections{c}{t} = {}; detections_inclocll{c}{t} = {};
          for m = 1:size(gt{c}{t},2)
              s = gt{c}{t}(m);
              if s.occl == 0 % Valid detection
                  detections{c}{t}{end+1} = [c t s.id s.pos];
              end
              detections_inclocll{c}{t}{end+1} = [c t s.id s.pos];
          end
          if ~isempty(detections{c}{t})
              detections{c}{t} = cell2mat(transpose(detections{c}{t}));
          end
          if ~isempty(detections_inclocll{c}{t})
              detections_inclocll{c}{t} = cell2mat(transpose(detections_inclocll{c}{t}));
          end
      end
    end

elseif strcmp(detect_mode,'CNN') == 1
    % TODO Get DeepPed detections



elseif strcmp(detect_mode,'ACF') == 1
    % TODO Get ACF detections

end
colors = {'Red','Blue','Black'};
num_ped_per_frame = cell(2,1);
% TODO Somehow show the number of valid detections on each frame for each camera overlayed as an actual line plot
figure; hold on;
for c = 1:length(cameras)
    num_ped_per_frame{c} = zeros(length(detections{c}),1);
    for t = 1:length(detections{c})
        num_ped_per_frame{c}(t) = size(detections{c}{t},1);
    end
    plot(1:length(detections{c}),num_ped_per_frame{c},colors{c});
end

[homographies, invhomographies] = loadHomographies(homography_directory,'hda', cameras);

ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'hda');
openfig(floor_image); hold on;
for i=1:length(cameras)
    drawPoly(ground_plane_regions{i},colors{i},0.5,false); % Draw regions
end
[overlap, ~, ~] = computeOverlap(ground_plane_regions);
drawPoly(overlap,colors{3},1.0,false);
%%===============================================================
% Convert all cam plane regions and store them if they don't exist already (this is the input to the tracker)
for i=1:length(cameras)
    fileID = fopen(['~/mct-bqp/hda_data/detections-gndplane/', num2str(cameras{i}), '.txt'],'w');
    for j=1:length(detections{i})
        gnd_detections{i}{j} = zeros(size(detections{i}{j},1), 9);
        for d=1:size(detections{i}{j},1)
            t = H(homographies{i}, [detections{i}{j}(d,4)+detections{i}{j}(d,6)/2; detections{i}{j}(d,5)+detections{i}{j}(d,7)]); % Get cam plane coordinates
            gnd_detections{i}{j}(d,1:7) = detections{i}{j}(d,1:7);

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
