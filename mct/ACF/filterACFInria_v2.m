function allDetections = filterACFInria_v2(allDetections)
    % Use the outlier tag existant in the original dataset
    % Inputs
    %  allDetections: all pedestrians identified by the detected

    % Output
    %  allDetections: all pedestrians identified by the detected

    setCaptureParams_hda_elevator;

    for id=1:length(cameras)
        allDetections{id}(allDetections{id}(:,7)==0, :) = []; %Remove those that have the occlusion bit to 1
        allDetections{id}(allDetections{id}(:,8)==999, :) = []; %Remove those that have the 999 annotation
        % Remove all other ped ID's besides the one found
        allDetections{id} = allDetections{id}(:,1:8);
    end
