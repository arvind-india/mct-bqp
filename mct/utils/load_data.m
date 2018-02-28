function gnd_detections = load_data(dataset)
    % load the detections file
    setDetectionParams_campus2;
    gnd_detections = cell(2,1);
    for id = 1:length(cameras)
        filename_g = ['/home/pedro/mct-bqp/' dataset '_data/detections-gndplane/' cameras{id} '.txt'];
        temp_g = csvread(filename_g);
        for i=1:size(temp_g,1)
            gnd_detections{id}{i} = temp_g(i,:);
        end
    end
    for id=1:length(cameras)
        gnd_detections{id} = cell2mat(gnd_detections{id}');
        gnd_detections{id} = (accumarray(gnd_detections{id}(:,2),(1:size(gnd_detections{id},1)).',[],@(x){gnd_detections{id}(x,:)},{}));
    end
end
