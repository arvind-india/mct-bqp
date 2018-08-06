% Set params of the detection
cameras = {17, 18, 19, 40, 50, 53, 54, 56, 57, 58, 59, 60};
homography_directory = '~/mct-bqp/data/hda/homographies';
hdaRootDirectory = '/media/pedro/mct/HDA_Dataset_V1.3';
num_cams = length(cameras);
detections = cell(num_cams,1);
detections_ocl = cell(num_cams,1); % NOTE We have all occluded detections included on this second variable

% Get homographies (we will need them)
[homographies, invhomographies] = loadHomographies(homography_directory,'hda',cameras);

% Get ground truth
gt = cell(num_cams,1);
for c=1:num_cams
  nmVbb = [hdaRootDirectory '/hda_annotations/cam' int2str(cameras{c}) '.txt'];
  annotations = vbb('vbbLoad', nmVbb);
  gt{c} = annotations.objLists;
  for t = 1:length(gt{c})
      detections{c}{t} = {}; detections_ocl{c}{t} = {};
      for m = 1:size(gt{c}{t},2)
          s = gt{c}{t}(m);
          if s.occl == 0 % Valid detection
              detections{c}{t}{end+1} = [c t s.id s.pos];
          end
          detections_ocl{c}{t}{end+1} = [c t s.id s.pos];
      end
      if ~isempty(detections{c}{t})
          detections{c}{t} = cell2mat(transpose(detections{c}{t}));
      end
      if ~isempty(detections_ocl{c}{t})
          detections_ocl{c}{t} = cell2mat(transpose(detections_ocl{c}{t}));
      end
  end
end

% Convert all cam detections and store them if they don't exist already (this is the input to the tracker)
for i=1:num_cams
    fileID = fopen(['~/mct-bqp/data/hda/hda_gt/', num2str(cameras{i}), '.txt'],'w');
    for j=1:length(detections{i})
        gnd_detections{i}{j} = zeros(size(detections{i}{j},1), 9);
        for d=1:size(detections{i}{j},1)
            t = H(homographies{i}, [detections{i}{j}(d,4)+detections{i}{j}(d,6)/2; detections{i}{j}(d,5)+detections{i}{j}(d,7)]); % Get cam plane coordinates
            gnd_detections{i}{j}(d,1:7) = detections{i}{j}(d,1:7);

            % NOTE this converts from detection indexes to per cam post filtering indexes. HDA already had some labels though...
            gnd_detections{i}{j}(d,3) = d;
            gnd_detections{i}{j}(d,8:9) = t; % x and y are now in the ground plane

            line = gnd_detections{i}{j}(d,:);
            formatSpec = '%d,%d,%d,%4.5f,%4.5f,%4.5f,%4.5f,%4.5f,%4.5f\n';
            fprintf(fileID,formatSpec,line(1),line(2),line(3),line(4),line(5),line(6),line(7),line(8),line(9));
        end
        %pause(0.1);
    end
end
