function runDebugDetection(cameraListImages,detector,rcnn_model,PersonW,PersonB,cl2,id,mode)
    setCaptureParams;
    setCalibrationVars;
    if mode == 0 %This is similar to deepPed_demo mode
        for i = 1:length(cameraListImages{id})
            img = imread(cameraListImages{id}{i});
            % detect possible pedestrians with LDCF
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
            I = find(scores(:) > score_threshold);
            scored_boxes = cat(2, bbs(I, :), scores(I));

            keep = nms(scored_boxes, NMS_maxoverlap);
            dets = scored_boxes(keep, :);
            dets(:,3) = dets(:,3) - dets(:,1); %x
            dets(:,4) = dets(:,4) - dets(:,2); %y

            % show the final obtained results
            figure;
            imshow(img);
            hold on
            drawBBs(dets,'g');
            hold off

            title(['Camera:' sprintf('%02d', id) ' Frame number:' sprintf('%d', i) ' BB threshold: ' sprintf('%f', score_threshold) ' Press x.']);
            waitforbuttonpress();
        end

    elseif mode == 1 %This picks a random frame from camera camID and shows the detections
        random_frame = randi(length(cameraListImages{id}),1,1);

        img = imread(cameraListImages{id}{random_frame});
        % detect possible pedestrians with LDCF
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
        I = find(scores(:) > score_threshold);
        scored_boxes = cat(2, bbs(I, :), scores(I));

        keep = nms(scored_boxes, NMS_maxoverlap);
        dets = scored_boxes(keep, :);
        dets(:,3) = dets(:,3) - dets(:,1); %x
        dets(:,4) = dets(:,4) - dets(:,2); %y

        % show the final obtained results
        figure;
        imshow(img);
        hold on
        drawBBs(dets,'g');
        title(['Camera:' sprintf('%02d', id) ' Frame number:' sprintf('%d', random_frame) ' BB threshold: ' sprintf('%f', score_threshold)]);

    end
