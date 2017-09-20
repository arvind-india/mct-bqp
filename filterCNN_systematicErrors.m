function allDetections = filterCNN_systematicErrors(allDetections)
    setCaptureParams_campus2;
    setDetectionParams_campus2;

    allDetections = {};
    %Load the detections file
    for id=1:2
        filename = ['allCNNdets_' sprintf('%02d', id) '.txt'];
        fileID = fopen(filename);
        file = textscan(fileID,'%d%d%f%f%f%f%f','Delimiter',',');
        fclose(fileID);
        temp = [double(file{1}) double(file{2}) file{3} file{4} file{5} file{6} file{7}];
        for i=1:size(temp,1)
            allDetections{id}{i} = temp(i,:);
        end
    end
    %Array of cells to matrix
    for id=1:2
        allDetections{id} = cell2mat(allDetections{id}');
        allDetections{id} = (accumarray(allDetections{id}(:,1),(1:size(allDetections{id},1)).',[],@(x){allDetections{id}(x,:)},{}));
    end
    %Try to correct systematic mistakes -> may use corrected versions for better training in the future
    systematic_error_threshold = [400 320];
    for id=1:2
        for frame=1:size(allDetections{id},1)
            if ~isempty(allDetections{id}{frame})
                m=1;
                while m <= size(allDetections{1,id}{frame,1},1)
                    val = allDetections{id}{frame}(m,4);
                    if val < systematic_error_threshold(id) + 20
                        allDetections{id}{frame}(m,:) = [];
                        m = m-1;
                    end
                    m = m+1;
                end
            end
        end
    end

    %Try to correct individual systematic mistakes that keep cropping up and are systematic
    systematic_errs = {};
    systematic_wrong_detections = {};
    systematic_wrong_detections{1} = [980.0 350.0; 835.0 602.0; 623.0 572.0; 678.0 430.0; 506.0 409.0];
    systematic_wrong_detections{2} = [815.0 365; 938 414; 7 350; 275 632; 808 392; 835 376; 801 360];
    systematic_boundary = [10 10]; %Means that it checks for any bb's in this boundary
    for id=1:2
        for frame=1:size(allDetections{id},1)
            if ~isempty(allDetections{id}{frame})
                m=1;
                while m <= size(allDetections{1,id}{frame,1},1)
                    xval = allDetections{id}{frame}(m,3);
                    yval = allDetections{id}{frame}(m,4);
                    %Check all systematic_wrong_detections to see if this detection is wrong
                    for wd=1:size(systematic_wrong_detections{id},1)
                        if xval < systematic_wrong_detections{id}(wd,1)+10 && xval > systematic_wrong_detections{id}(wd,1)-10 && yval < systematic_wrong_detections{id}(wd,2)+10 && yval > systematic_wrong_detections{id}(wd,2)-10
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
