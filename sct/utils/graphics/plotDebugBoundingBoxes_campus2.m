function plotDetections_campus2(cameraListImages,allDetections,startframe)
    for id=1:2
        figure
        index = 1;
        offset = startframe+3;
        for fi=startframe:offset
        	subplot(2,2,index), subimage(imread(cameraListImages{id}{fi}));
            for frame=1:size(allDetections{id},1)
                if allDetections{id}{frame}(1) == fi
                    drawBBs(allDetections{id}{frame}(:,3:7),'g');
                end
            end
            index = index + 1;
        end
    end
