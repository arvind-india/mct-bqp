function allDetections = filterCNN_systematicErrors(allDetections, inplanes)
    setCaptureParams_campus2;
    setDetectionParams_campus2;
    %---------------------------------------------------------------------------
    %allDetections = cell(length(cameras),1);
    %Load the detections file
    for id=1:length(cameras)
        filename = ['~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/campus2_data/CPU/CNNdets_' sprintf('%02d', id) '.txt'];
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

    %---------------------------------------------------------------------------
    %Try to correct systematic mistakes -> may use corrected versions for better training in the future
    %systematic_error_threshold = [400 320];
    for id=1:length(cameras)
        for frame=1:size(allDetections{id},1)
            if ~isempty(allDetections{id}{frame})
                m=1;
                while m <= size(allDetections{id}{frame},1)
                    val = allDetections{id}{frame}(m,3:4);
                    if inpoly(val,inplanes{id}) == 0
                        allDetections{id}{frame}(m,:) = [];
                        m = m-1;
                    end
                    m = m+1;
                end
            end
        end
    end

    %---------------------------------------------------------------------------
    %Try to correct individual systematic mistakes that keep cropping up and are systematic
    systematic_wrong_detections = {};
    filename = '~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/campus2_data/cam_alameda_systematicFP.txt';
    delimiterIn = ' ';
    headerlinesIn = 1;
    systematic_wrong_detections{1} = importdata(filename,delimiterIn,headerlinesIn); systematic_wrong_detections{1} = systematic_wrong_detections{1}.data;
    filename = '~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/campus2_data/cam_central_systematicFP.txt';
    systematic_wrong_detections{2} = importdata(filename,delimiterIn,headerlinesIn); systematic_wrong_detections{2} = systematic_wrong_detections{2}.data;
    error_variance = [10 10]; %Means that it checks for any bb's within this range (in pixels)
    for id=1:length(cameras)
        for frame=1:size(allDetections{id},1)
            if ~isempty(allDetections{id}{frame})
                m=1;
                while m <= size(allDetections{id}{frame},1)
                    xval = allDetections{id}{frame}(m,3);
                    yval = allDetections{id}{frame}(m,4);
                    %Check all systematic_wrong_detections to see if this detection is wrong
                    for wd=1:size(systematic_wrong_detections{id},1)
                        if xval < systematic_wrong_detections{id}(wd,1) + error_variance(1) && xval > systematic_wrong_detections{id}(wd,1) - error_variance(1) && yval < systematic_wrong_detections{id}(wd,2) + error_variance(2) && yval > systematic_wrong_detections{id}(wd,2) - error_variance(2)
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
