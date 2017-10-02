function plotDebugBoundingBoxes_hda(imageSequenceFolder,allDetections,startframe)
    for id=1:2
        figure
        index = 1;
        offset = startframe+3;
        for fi=startframe:offset
            %subplot(2,2,index), subimage(imread(cameraListImages{id}{fi}));
            subplot(2,2,index);
            for i=1:size(allDetections{id},1)
                if allDetections{id}(i,2) == fi
                    drawBBs(allDetections{id}(i,3:7),'g');
                end
            end
            index = index + 1;
        end
    end
