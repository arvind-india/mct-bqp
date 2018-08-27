function gnd_detections = loadDetections(dataset, cameras)
    % load the detections file
    gnd_detections = cell(length(cameras),1);
    for id = 1:length(cameras)
        if strcmp(dataset,'ucla')
            filename_g = ['/home/pedro/mct-bqp/data/' dataset '/groundtruth/view-GL' num2str(cameras{id}) '.txt'];
        else
            filename_g = ['/home/pedro/mct-bqp/data/' dataset '/groundtruth/' num2str(cameras{id}) '.txt'];
        end
        temp_g = csvread(filename_g);

        gnd_detections{id} = temp_g;
        if strcmp(dataset,'hda')
            gnd_detections{id}(:,2) = gnd_detections{id}(:,2) + 1; % NOTE Because of matlab
            gnd_detections{id}(:,1) = id; % NOTE Needed because of using global gndtruth
        end
    end
    for id=1:length(cameras)
        if strcmp(dataset,'hda')
            %gnd_detections{id} = cell2mat(gnd_detections{id}');
            gnd_detections{id} = (accumarray(gnd_detections{id}(:,2),(1:size(gnd_detections{id},1)).',[],@(x){gnd_detections{id}(x,:)},{}));
        else
            gnd_detections{id} = (accumarray(gnd_detections{id}(:,7),(1:size(gnd_detections{id},1)).',[],@(x){gnd_detections{id}(x,:)},{}));
        end
    end

end
