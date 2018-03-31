function gnd_detections = load_data(dataset, cameras)
    % load the detections file
    gnd_detections = cell(2,1);
    if strcmp(dataset, 'mini-campus2')
        % Does the gndplane-detections exist?

        % TODO Load the annotated camera plane
        for id = 1:length(cameras)
            filename_c = ['/home/pedro/mct-bqp/' dataset '_data/detections-camplane/' num2str(cameras{id}) '.txt'];
            temp_c = csvread(filename_c);
            for i=1:size(temp_c,1)
                cam_detections{id}{i} = temp_c(i,:);
            end
            cam_detections{id} = cell2mat(cam_detections{id}');




            for j=1:length(cam_detections{i})
                t = H_alt(homographies{i}, [detections{i}{j}(d,3)+detections{i}{j}(d,5)/2 detections{i}{j}(d,4)+detections{i}{j}(d,6)]); % Get cam plane coordinates
            end

        end

    else
        for id = 1:length(cameras)
            filename_g = ['/home/pedro/mct-bqp/' dataset '_data/detections-gndplane/' num2str(cameras{id}) '.txt'];
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
end
