function allDetections = CNNdetect(cameraListImages,sample_size)
    setCalibrationVars
    setCaptureParams
    [SVM detector rcnn_model cl2 PersonW PersonB] = loadCNN(LDCF_cascThr,LDCF_cascCal,LDCF_rescale);

    %==========================================================
    %Just for now, working with this
    allDetections = {}; %Run detections on the whole dataset for both camera and store detections per frame on an outside file

    for id = 1:size(cameras,2)
        if ~exist(['allCNNdets_' sprintf('%02d', id) '.txt'], 'file')
            for i = 1:sample_size
                disp(['Frame ' sprintf('%d',i) '.Percentage done: ' sprintf('%f',i/sample_size)]);
                img = imread(cameraListImages{id}{i});
                % detect possible pedestrians with LDCF1
                bbs = acfDetect(img,detector);
                dt_ldcf = bbs;

                % evaluate BBs retrieved by LDCF with our finetuned AlexNet
                bbs(:,3) = bbs(:,1) + bbs(:,3);
                bbs(:,4) = bbs(:,2) + bbs(:,4);
                bbs(:,5) = [];
                feat = rcnn_features(img, bbs, rcnn_model);
                scores_cnn = feat*PersonW + PersonB;

                % use second level SVM
                scores = [dt_ldcf(:,5) scores_cnn]*cl2.W+cl2.b;

                % discard BBs with too low score and apply NMS
                I = find(scores(:) > score_threshold(id));
                scored_boxes = cat(2, bbs(I, :), scores(I));

                keep = nms(scored_boxes, NMS_maxoverlap(id));
                dets = scored_boxes(keep, :);
                dets(:,3) = dets(:,3) - dets(:,1); % x
                dets(:,4) = dets(:,4) - dets(:,2); % y
                allDetections{id}{i} = dets;

            end
            %Transform detections to a file so we can load faster in the future
        	fileID = fopen(['allCNNdets_' sprintf('%02d', id) '.txt'],'w');
        	formatSpec = '%d,%d,%4.5f,%4.5f,%4.5f,%4.5f,%1.5f\n';
        	for i=1:sample_size
                for k=1:size(allDetections{id}{i},1)
                    fprintf(fileID,formatSpec,i,k,allDetections{id}{i}(k,1),allDetections{id}{i}(k,2),allDetections{id}{i}(k,3),allDetections{id}{i}(k,4),allDetections{id}{i}(k,5));
                end
        	end
        else
            % load the file
        	filename = ['allCNNdets_' sprintf('%02d', id) '.txt'];
        	fileID = fopen(filename);
        	file = textscan(fileID,'%d%d%f%f%f%f%f','Delimiter',',');
        	fclose(fileID);
        	temp = [double(file{1}) double(file{2}) file{3} file{4} file{5} file{6} file{7}];
        	for i=1:size(temp,1)
                allDetections{id}{i} = temp(i,:);
            end
        end
    end
