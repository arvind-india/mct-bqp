function plotDetectionsCameraSpace_hda(allDetections)
    setCaptureParams_hda_elevator;
    figure
    for id=1:2
        subplot(2,1,id);
        hold on
        scatter(allDetections{id}(:,3)+0.5*allDetections{id}(:,5),allDetections{id}(:,4)+0.5*allDetections{id}(:,6),10,'filled','MarkerFaceColor','black');
        axis([0 resolutions(id,1) 0 resolutions(id,2)])
        set(gca,'Ydir','reverse');
        title(['Camera ' sprintf('%d',cameras{id})])
        hold off
    end
