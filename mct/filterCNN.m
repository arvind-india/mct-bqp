function allDetections = filterCNN(~,inplanes)
    % Given a list of detections and visibility camera regions
    % Inputs
    %  allDetections: all pedestrians identified so far
    %  inplanes: representative planes of each camera

    % Output
    %  allDetections: filtered pedestrian identifications

    setCaptureParams_campus2;
    setDetectionParams_campus2;

    allDetections = cell(length(cameras),1);
    %Load the detections file
    for id=1:length(cameras)
        if use_GPU == 1
            filename = ['~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/campus2_data/GPU/CNNdets_' sprintf('%02d', id) '.txt'];
        else
            filename = ['~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/campus2_data/CPU/CNNdets_' sprintf('%02d', id) '.txt'];
        end
        fileID = fopen(filename);
        file = textscan(fileID,'%d%d%f%f%f%f%f','Delimiter',',');
        fclose(fileID);
        temp = [double(file{1}) double(file{2}) file{3} file{4} file{5} file{6} file{7}];
        for i=1:size(temp,1)
            allDetections{id}{i} = temp(i,:);
        end
    end
    %Array of cells to matrix
    for id=1:length(cameras)
        allDetections{id} = cell2mat(allDetections{id}');
        allDetections{id} = (accumarray(allDetections{id}(:,1),(1:size(allDetections{id},1)).',[],@(x){allDetections{id}(x,:)},{}));
    end

    %Try to correct systematic mistakes -> may use corrected versions for better training in the future
    for id=1:length(cameras)
        for frame=1:size(allDetections{id},1)
            if ~isempty(allDetections{id}{frame})
                m=1;
                while m <= size(allDetections{id}{frame},1)
                    val = allDetections{id}{frame}(m,3:4);
                    if inpoly(val,inplanes{id})
                        allDetections{id}{frame}(m,:) = [];
                        m = m-1;
                    end
                    m = m+1;
                end
            end
        end
    end

    %Try to correct individual systematic mistakes that keep cropping up and are systematic like trees or lamp-posts
    systematic_wrong_detections = {};
    systematic_wrong_detections{1} = [980.0 350.0; 835.0 602.0; 623.0 572.0; 678.0 430.0; 506.0 409.0];
    systematic_wrong_detections{2} = [815.0 365; 938 414; 7 350; 275 632; 808 392; 835 376; 801 360];
    systematic_boundary = [10 10]; %Means that it checks for any bb's in this boundary
    for id=1:length(cameras)
        for frame=1:size(allDetections{id},1)
            if ~isempty(allDetections{id}{frame})
                m=1;
                while m <= size(allDetections{id}{frame},1)
                    xval = allDetections{id}{frame}(m,3);
                    yval = allDetections{id}{frame}(m,4);
                    %Check all systematic_wrong_detections to see if this detection is wrong
                    for wd=1:size(systematic_wrong_detections{id},1)
                        if xval < systematic_wrong_detections{id}(wd,1)+systematic_boundary(1) && xval > systematic_wrong_detections{id}(wd,1)-systematic_boundary(1) && yval < systematic_wrong_detections{id}(wd,2)+systematic_boundary(2) && yval > systematic_wrong_detections{id}(wd,2)-systematic_boundary(2)
                            allDetections{id}{frame}(m,:) = [];
                            m = m-1;
                            break;
                        end
                    end
                    m=m+1;
                end
            end
        end
    end
