function plotDebugBoundingBoxes(cameraListImages,allDetections,startframe,dataset)
    % Draw debug bounding boxes
    % Inputs
    %  cameraListImages: list of camera images for all cameras
    %  allDetections: all pedestrians identified by the detected
    %  startframe: frame at which to display bounding boxes, displays the next 3
    %  dataset: tag for the dataset

    if strcmp(dataset,'campus_2')
        for id=1:2
            index = 1;
            %startframe = 28 % Debug
            figure
            offset = startframe+3;
            for fi=startframe:offset
            	subplot(2,2,index), subimage(imread(cameraListImages{id}{fi}));
                %imshow(imread(cameraListImages{id}{fi})) % Debug
                for frame=1:size(allDetections{id},1)
                    if allDetections{id}{frame}(1) == fi
                        drawBBs(allDetections{id}{frame}(:,3:7),'g');
                    end
                end
                index = index + 1;
            end
        end
    end
    if strcmp(dataset,'hda')
        for id=1:2
            index = 1;
            figure
            offset = startframe+3;
            for fi=startframe:offset
                subplot(2,2,index), subimage(imread(cameraListImages{id}{fi}));
                %subplot(2,2,index);
                for i=1:size(allDetections{id},1)
                    if allDetections{id}(i,2) == fi
                        drawBBs(allDetections{id}(i,3:7),'g');
                    end
                end
                index = index + 1;
            end
        end
    end
