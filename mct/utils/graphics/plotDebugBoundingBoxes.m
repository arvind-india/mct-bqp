function plotDebugBoundingBoxes(cameraListImages,allDetections,startframe,dataset,id)
    % Draw debug bounding boxes
    % Inputs
    %  cameraListImages: list of camera images for all cameras
    %  allDetections: all pedestrians identified by the detected
    %  startframe: frame at which to display bounding boxes, displays the next 3
    %  dataset: tag for the dataset

    if strcmp(dataset,'campus_2') == 1
        index = 1;
        %startframe = 28 % Debug
        offset = startframe + 3;
        for fi=startframe:offset
        	subplot(2,2,index), subimage(imread(cameraListImages{id}{fi}));
            %imshow(imread(cameraListImages{id}{fi})) % Debug
            title(['Frame: ' sprintf('%d',fi)]);
            for frame=1:size(allDetections{id},1)
                if allDetections{id}{frame}(1) == fi
                    drawBBs(allDetections{id}{frame}(:,3:7),'g',dataset);
                end
            end
            index = index + 1;
        end
    end
    if strcmp(dataset,'hda') == 1
        index = 1;
        offset = startframe + 3;
        for fi=startframe:offset
            subplot(2,2,index), subimage(imread(cameraListImages{id}{fi}));
            %subplot(2,2,index);
            title(['Frame: ' sprintf('%d',fi)]);
            for i=1:size(allDetections{id},1)
                if allDetections{id}(i,2) == fi
                    drawBBs(allDetections{id}(i,3:7),'g',dataset);
                end
            end
            index = index + 1;
        end
    end
