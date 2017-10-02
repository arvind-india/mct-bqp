setCalibrationVars
setCaptureParams
homographies_struct = load('~/Campus_II/homography_campus_II.mat');
homographies{1} = homographies_struct.TFORM_alameda.tdata.T;
homographies{2} = homographies_struct.TFORM_central.tdata.T;
invhomographies{1} = homographies_struct.TFORM_alameda.tdata.Tinv;
invhomographies{2} = homographies_struct.TFORM_central.tdata.Tinv;
%Camera Alameda to Central matrix
Hac = invhomographies{2}*homographies{1};
%Camera Central to Alameda matrix
Hca = invhomographies{1}*homographies{2};
cameraListImages = {};
for i=1:size(cameras,2)
    cameraListImages{i} = bbGt('getFiles',{image_directories{i}});
end

c_h = {};
sigma = 400;
allDetections = CNNdetect(cameraListImages,200);%Sample size
allDetections = parseDetections(allDetections);

%Load the candidate file
allCandidates_cam1 = {};
filename = ['allCandidates_cam1.txt'];
fileID = fopen(filename);
file = textscan(fileID,'%f%f%f%f','Delimiter',',');
fclose(fileID);
temp = [file{1} file{2} file{3} file{4}];
for i=1:size(temp,1)
    allCandidates_cam1{i} = temp(i,:);
end
allCandidates_cam1 = cell2mat(allCandidates_cam1');
n = 5;
allCandidates_cam1 = mat2cell(allCandidates_cam1,repmat(25,n,1),4)
f = 2;
allDetections_cam1 = allDetections{1}{f};
allDetections_cam2 = allDetections{2}{f};

cam_id = 1;
cam1_n = size(allDetections_cam1,1);
cam2_n = size(allDetections_cam2,1);
cam1_k = 25; %5x5 kernels
cam23_k = 25; %5x5 kernels
%Camera Alameda
if cam_id == 1

    %DEBUG: This is for debug
    target_cam2tocam1 = {};
    for m = 1:size(allDetections{2}{2},1)
        %allDetections{1}{f}(neighbour,:);
        %TODO: this only needs to be done once
        xhomog_pos = allDetections{2}{2}(m,:);
        xhomog = [xhomog_pos(3:4) 1];
        xhomog = homographies{1}*xhomog';
        lambda = xhomog(3);
        %xhomog = xhomog./lambda;
        %xhomog = invhomographies{1}*xhomog;
        target_cam2tocam1{end+1} = xhomog;
    end

    %For all candidates of targets in camera Alameda
    for j = 1:cam1_n
        hcandidate_weights = zeros(cam1_k,1);
        for i = 1:cam1_k
            %allbbs{j}
            candidate_pos = allCandidates_cam1{j}(i,:);
            x_i = candidate_pos(1:2);
            %For all targets in camera Central
            H_weight = 0.0;
            for m = 1:cam2_n
                %allDetections{1}{f}(neighbour,:);
                %TODO: this only needs to be done once
                xhomog_pos = allDetections_cam2(m,:);
                xhomog = [xhomog_pos(3:4) 1];
                xhomog = Hca*xhomog';
                xhomog(1) = xhomog(1)/xhomog(3);
                xhomog(2) = xhomog(2)/xhomog(3);
                val = exp(-sqrt((x_i(1)-xhomog(1))^2 + (x_i(2)-xhomog(2))^2)/sigma^2);
                H_weight = H_weight + val;
            end
            hcandidate_weights(i) = H_weight/cam2_n;
        end
        c_h{j} = hcandidate_weights;
    end



%Camera Central
elseif cam_id == 2
    %For all candidates of targets in camera Alameda
    for j = 1:cam2_n
        hcandidate_weights = zeros(cam2_k,1);
        for i = 1:cam2_k
            %allbbs{j}
            candidate_pos = allCandidates_cam2{j}(i,:);
            x_i = candidate_pos(1:2);
            %For all targets in camera Central
            H_weight = 0.0;
            for m = 1:cam1_n
                %allDetections{1}{f}(neighbour,:);
                xhomog_pos = allDetections_cam1(m,:);
                xhomog = [xhomog_pos(3:4) 1];
                xhomog = Hac*xhomog';
                xhomog(1) = xhomog(1)/xhomog(3);
                xhomog(2) = xhomog(2)/xhomog(3);
                val = exp(-sqrt((x_i(1)-xhomog(1))^2 + (x_i(2)-xhomog(2))^2)/sigma^2);
                H_weight = H_weight + val;
            end
            hcandidate_weights(i) = H_weight;
        end
        c_h{j} = hcandidate_weights;
    end
end

figure
hold on
for j = 1:cam1_n
    plot(1:cam1_k,c_h{j},'--*');
end
legend('1','2','3','4','5')

figure
imshow(imread(cameraListImages{1}{2}));
hold on
%Plot all candidates positions in camera 1
for j=1:cam1_n
    scatter(allCandidates_cam1{j}(:,1),allCandidates_cam1{j}(:,2),'filled');
end
set(gca,'Ydir','reverse');
%Plot all target from camera 2 converted to the plane of camera 1
for targ=1:cam2_n
    scatter(target_cam2tocam1{targ}(1),target_cam2tocam1{targ}(2),'filled');
end

figure
imshow(imread(cameraListImages{2}{2}));
hold on
for j=1:cam2_n
    scatter(allDetections_cam2(j,3),allDetections_cam2(j,4),'filled');
end
set(gca,'Ydir','reverse');
