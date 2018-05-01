setDetectionParams_campus2;
dataset = 'mini-campus2';

% Load homographies, these homographies where obtained via the use of cp2tform
[homographies, invhomographies] = loadHomographies(homography_directory,'campus_2', cameras); % Defined in global variables

% TODO Load the annotated camera plane
cam_detections = cell(length(cameras),1); gnd_detections = cell(length(cameras),1);
for id = 1:length(cameras)
    filename_c = ['~/mct-bqp/' dataset '_data/detections-camplane/' num2str(cameras{id}) '.txt'];
    temp_c = csvread(filename_c);
    for i = 1:size(temp_c,1)
        cam_detections{id}{i} = temp_c(i,:);
    end
    cam_detections{id} = cell2mat(cam_detections{id}');

    for j=1:size(cam_detections{id},1)
        % Get cam plane coordinates and convert them
        t = H_alt(homographies{id}, [cam_detections{id}(j,4) + cam_detections{id}(j,6)/2 cam_detections{id}(j,5) + cam_detections{id}(j,7)]);
        gnd_detections{id}(j,:) = [cam_detections{id}(j,:) t];
    end
end
%===================================================
% Convert all cam plane regions and store them if they don't exist already (this is the input to the tracker)
figure; hold on;
for i=1:length(cameras)
    fileID = fopen(['~/mct-bqp/' dataset '_data/detections-gndplane/', cameras{i} '.txt'],'w');
    for j=1:size(gnd_detections{i},1)
        line = gnd_detections{i}(j,:);
        if i == 1
            plot(line(8),line(9),'r+');
        end
        if i == 2
            plot(line(8),line(9),'b+');
        end
        formatSpec = '%d,%d,%d,%4.5f,%4.5f,%4.5f,%4.5f,%4.5f,%4.5f\n';
        fprintf(fileID,formatSpec,line(1),line(2),line(3),line(4),line(5),line(6),line(7),line(8),line(9));
        pause(0.1);
    end
end
