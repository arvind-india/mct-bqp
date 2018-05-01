function gnd_detections = load_data(dataset, cameras, subset)
    % load the detections file
    gnd_detections = cell(length(cameras),1);
    for id = 1:length(cameras)
        filename_g = ['/home/pedro/mct-bqp/' dataset '_data/' subset '/detections-gndplane/' num2str(cameras{id}) '.txt'];
        temp_g = csvread(filename_g);
        gnd_detections{id} = temp_g;
        gnd_detections{id}(:,2) = gnd_detections{id}(:,2) + 1; % NOTE Because of matlab
    end
    for id=1:length(cameras)
        %gnd_detections{id} = cell2mat(gnd_detections{id}');
        gnd_detections{id} = (accumarray(gnd_detections{id}(:,2),(1:size(gnd_detections{id},1)).',[],@(x){gnd_detections{id}(x,:)},{}));
        if strcmp('mini-campus2',dataset)
            gnd_detections{id} = gnd_detections{id}(~cellfun('isempty',gnd_detections{id}));
        end
    end

end
