function plotTrajectoriesCameraSpace(cameraListImages,allDetections)
    setCaptureParams
    figure
    for id=1:2
        subplot(2,1,id);
        hold on
        %subimage(imread(cameraListImages{id}{1}));
        for f=1:size(allDetections{id},1)
            scatter(allDetections{id}{f}(:,3)+0.5*allDetections{id}{f}(:,5),allDetections{id}{f}(:,4)+0.5*allDetections{id}{f}(:,6),10,'filled','MarkerFaceColor','black');
        end
        axis([0 size(imread(cameraListImages{1}{1}),2) 0 size(imread(cameraListImages{1}{1}),1)])
        set(gca,'Ydir','reverse');
        title(['Camera ' sprintf('%s',cameras{id})])
        hold off
    end
