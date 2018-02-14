function allDetections = CNNdetect(cameraListImages,sample_size)
    % Given a list of images and a sample size
    % Inputs
    %  allFG_time: Get all frames with their respective ground truth

    % Output
    %  allDetections: all detections from the CNN
    setDetectionParams_campus2;

    %==========================================================
    %Just for now, working with this
    allDetections = cell(length(cameras),1); % Run detections on the whole dataset for both camera and store detections per frame on an outside file

    for id = 1:length(cameras)
        
        directory_not_exists = 0;
        if use_GPU(id) == 1
            if ~exist([gpu_results sprintf('%02d', id) '.txt'], 'file')
                directory_not_exists = 1;
            end
        else
            if ~exist([cpu_results sprintf('%02d', id) '.txt'], 'file')
                directory_not_exists = 1;
            end
        end
        if directory_not_exists == 1
            %==========================================================
            [~, detector, rcnn_model, cl2, PersonW, PersonB] = loadCNN(LDCF_cascThr(id),LDCF_cascCal(id),LDCF_rescale(id), use_GPU(id));
            %==========================================================
            allDetections{id} = cell(sample_size,1);
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
                scores = [dt_ldcf(:,5) scores_cnn] * cl2.W + cl2.b;

                % discard BBs with too low score and apply NMS
                I = find(scores(:) > score_threshold(id));
                scored_boxes = cat(2, bbs(I, :), scores(I));

                keep = nms(scored_boxes, NMS_maxoverlap(id));
                dets = scored_boxes(keep, :);
                dets(:,3) = dets(:,3) - dets(:,1); % x
                dets(:,4) = dets(:,4) - dets(:,2); % y
                allDetections{id}{i} = dets;

            end
            disp(['Camera ' sprintf('%d',id) ' is done']);
            %Transform detections to a file so we can load faster in the future
            if use_GPU(id) == 1
        	       fileID = fopen([gpu_results sprintf('%02d', id) '.txt'],'w');
            else
        	       fileID = fopen([cpu_results sprintf('%02d', id) '.txt'],'w');
            end
        	formatSpec = '%d,%d,%4.5f,%4.5f,%4.5f,%4.5f,%1.5f\n';
        	for i=1:sample_size
                for k=1:size(allDetections{id}{i},1)
                    fprintf(fileID,formatSpec,i,k,allDetections{id}{i}(k,1),allDetections{id}{i}(k,2),allDetections{id}{i}(k,3),allDetections{id}{i}(k,4),allDetections{id}{i}(k,5));
                end
        	end

        else
            % load the file
            if use_GPU(id) == 1
        	       filename = [gpu_results sprintf('%02d', id) '.txt'];
            else
        	       filename = [cpu_results sprintf('%02d', id) '.txt'];
            end
            fileID = fopen(filename);
        	file = textscan(fileID,'%d%d%f%f%f%f%f','Delimiter',',');
        	fclose(fileID);
        	temp = [double(file{1}) double(file{2}) file{3} file{4} file{5} file{6} file{7}];
            for i=1:size(temp,1)
                allDetections{id}{i} = temp(i,:);
            end
        end
    end
    % Get the currect parallel pool and free the memory
    poolobj = gcp('nocreate'); delete(poolobj);
end

function [SVM, detector, rcnn_model, cl2, PersonW, PersonB] = loadCNN(LDCF_cascThr,LDCF_cascCal,LDCF_rescale, use_GPU)
    % load and adjust the LDCF detector
    % See toolbox/detector/acfModify and acfDetect for more info
    load('toolbox/detector/models/LdcfCaltechDetector.mat');
    % Show the default options
    % opts=acfTrain()

    pModify = struct('cascThr',LDCF_cascThr,'cascCal',LDCF_cascCal,'rescale',LDCF_rescale); %stride not included yet
    % detector   - detector trained via acfTrain

    % TODO: Probably should define detector here?

    detector = acfModify(detector, pModify);

    % load the trained SVM
    SVM = load('data/rcnn_models/DeepPed/SVM_finetuned_alexnet.mat');
    PersonW = SVM.W; %Feature weights for scoring
    %PersonW = SVM.W(1:end/2; %Dirty hack, the only real solution to this should be to get a better GPU than this crappy GTX960
    PersonB = SVM.b; %Constant scoring factor

    % load the trained svm of level 2
    cl2 = load('data/rcnn_models/DeepPed/SVM_level2.mat');

    %load the finetuned AlexNet
    rcnn_model = rcnn_load_model('data/rcnn_models/DeepPed/finetuned_alexNet.mat', use_GPU); %0 is CPU, 1 is GPU, the file is the model file
end
