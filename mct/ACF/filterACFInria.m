function allDetections = filterACFInria(allDetections)
    % Use the outlier tag existant in the original dataset
    % Inputs
    %  allDetections: all pedestrians identified by the detected

    % Output
    %  allDetections: all pedestrians identified by the detected

    setDetectionParams_hda_elevator;

    for id=1:length(cameras)
        allDetections{id}(allDetections{id}(:,7)==0, :) = []; %Remove those that have the occlusion bit to 1
        for i=1:size(allDetections{id},1)
            % Find the first pedestrian ID that is not 999
            for cand_id = 8:size(allDetections{id},2)
                if allDetections{id}(i,cand_id) ~= 999
                    best_candidate = allDetections{id}(i,cand_id);
                    break;
                end
            end
            % Assign to column 8 the first ped ID
            allDetections{id}(i,8) = best_candidate;
        end
        % Remove all other ped ID's besides the one found
        allDetections{id} = allDetections{id}(:,1:8);
    end
