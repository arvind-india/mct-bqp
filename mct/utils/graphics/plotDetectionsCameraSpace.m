function plotDetectionsCameraSpace(cameraListImages,allDetections,dataset)
    % Plot camera space detections
    % Inputs
    %  cameraListImages: list of camera images for all cameras
    %  allDetections: all pedestrians identified by the detected
    %  dataset: tag for the dataset
    
    if strcmp(dataset,'campus_2')
        setCaptureParams_campus2;
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
    end
    if strcmp(dataset, 'hda')
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
    end
