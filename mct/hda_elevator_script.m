% Set params of the capture
setCaptureParams_hda_elevator;
% Set params of the calibration
setDetectionParams_hda_elevator;
% Set parameters for the tracker
setTrackerParams;
% Pick if you want to display debug images or not
show_images = 'on';
set(0,'DefaultFigureVisible',show_images);
%%=========================================================
allDetections = ACFdetect();
% In allDetections get the first ground_truth pedestrian id that is not 999 and filter using the occlusion bit
allDetections = filterACFInria(allDetections); % If you want more than 1 ground_truth label for each ped you can change it here
%%=========================================================
% Plot detections of all peds in both cameras (we consider that peds are represented by the middle of their BB's)
cameraListImages = cell(length(cameras),1);
%plotDetectionsCameraSpace(allDetections,cameraListImages,'hda');
%%=========================================================
% Load homographies, these homographies where obtained via the use of cp2tform
[homographies, invhomographies] = loadHomographies(homography_directory,'hda'); % Defined in global variables
% Plot regions -- we try to be as permissive as possible in these regions while keeping them simple
% Alternatively we were provided with very detailed regions from the people who own the datasets, we can use both
%camera_plane_regions{1} = [59.7037 796.9403; 503.3571 353.2869; 1015.9567 375.7695; 1011.4602 798.4391];
%camera_plane_regions{2} = [4.2471 796.4391; 5.7459 491.1792; 908.0410 399.7506; 1098.3923 798.4391];
outlier_removal_regions{1} = load(strcat('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/homographies/visibility_points_image_',num2str(cameras{1}),'.mat'));
outlier_removal_regions{2} = load(strcat('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/homographies/visibility_points_image_',num2str(cameras{2}),'.mat'));
outlier_removal_regions{1} = outlier_removal_regions{1}.t; outlier_removal_regions{2} = outlier_removal_regions{2}.t;
%%=========================================================
%openfig('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/7th_floor_ground_plane_reference_frame_map.fig');
%hold on;
ground_plane_regions = computeGroundPlaneRegions(outlier_removal_regions, homographies, length(cameras), 'hda');
% Plot pedestrians
plotDetectionsGroundPlane(allDetections,homographies, ground_plane_regions, 'show_outliers', 'hda');
% Draw regions
colors = {'Red','Green'};
for i=1:length(cameras)
    %drawPoly(ground_plane_regions{i},colors{i},0.5,false);
end
% Plot overlap of camera regions
[overlap, ~, ~] = computeOverlap(ground_plane_regions); % Performed in the pixel plane
%drawPoly(overlap,'Black',1.0,false);
%%=========================================================
% Group allDetections in terms of frames for both cameras
ground_truth = cell(length(cameras),1);
for i=1:length(cameras)
    ground_truth{i} = accumarray(allDetections{i}(:,8),(1:size(allDetections{i},1)).',[],@(x){allDetections{i}(x,:)},{});
end
for i=1:length(cameras)
    allDetections{i} = accumarray(allDetections{i}(:,2),(1:size(allDetections{i},1)).',[],@(x){allDetections{i}(x,:)},{});
end
%%=========================================================
% Get camera images
data_folders = '~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/cam';
% NOTE These two following values, especially sample size are likely to be altered for the actual tracking problem for debug reasons
sample_size = 500; % Number of frames
start_frame = 1000; % Frames to start collecting images
for i=1:length(cameras)
    if exist(data_folders(i),'dir')
        foldercontent = dir(strcat(data_folders,num2str(cameras{i})));
        if numel(foldercontent) == 2
            %# folder exists and is empty
            seq2img(cameras{i},start_frame, sample_size);
        end
    end
    %cameraListImages{i} = cell(sample_size,1);
    for j=start_frame:(start_frame + sample_size)
        cameraListImages{i}{j} = imread(strcat(data_folders, num2str(cameras{i}), '/', num2str(j), '.png'));
    end
end
%%=========================================================
% Plot 4 sample_size images to show detections (with bounding boxes)
%plotDebugBoundingBoxes(cameraListImages,allDetections,start_frame,'hda');
%%=========================================================
% Build POM
ref_img = imread('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/ref_58.png');
test_img = imread('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/test_img_58.png');
test_frame = 450;
%[img_diff, bin_img] = convertBB2bin_img(allDetections{2}{test_frame}, test_img, ref_img);
%figure
%imshow(out);
%figure
%imshow(bin_img);

gplane = imread('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/7th_floor_ground_plane_reference_frame_map.png');
gplane = imcrop(gplane,[198,167,100,100]);
figure
imagesc([.5 .5],[.5 .5],gplane)

%imshow(gplane);
hold on;
drawGrid(gplane,10,'hda');
figure
%%=========================================================
cam_ground_truth = ground_truth;
% Draw a ground truth for an experiment
for id=1:length(cameras)
    for p=1:size(ground_truth{id},1)
        if ~isempty(ground_truth{id}{p})
            pedpos = horzcat(ground_truth{id}{p}(:,3)+ground_truth{id}{p}(:,5)/2, ground_truth{id}{p}(:,4)+ground_truth{id}{p}(:,6));
            transfpos = zeros(size(pedpos,1),2);
            for i=1:size(pedpos,1)
                pts = pedpos(i,:);
                u = pts(1);
                v = pts(2);
                o = 1;
                uvo = [u; v; o];
                new_pts = homographies{id}*uvo;

                new_pts = [new_pts(1)./new_pts(3) new_pts(2)./new_pts(3)];
                transfpos(i,:) = new_pts;
            end
            ground_truth{id}{p}(:,9:10) = transfpos;
        end
    end
end
ped1 = 52;
ped2 = 15;
cam_57_ped52 = ground_truth{1}{ped1}(5:137,:);
hold on;
cam_58_ped52 = ground_truth{2}{ped1}(6:108,:);
scatter(cam_57_ped52(:,9), cam_57_ped52(:,10),4,'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red')); % Red
scatter(cam_58_ped52(:,9), cam_58_ped52(:,10),4,'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange')); % Orange

cam_57_ped15 = ground_truth{1}{ped2}(26:92,:);
cam_58_ped15 = ground_truth{2}{ped2}(28:86,:);
scatter(cam_57_ped15(:,9), cam_57_ped15(:,10),4,'MarkerFaceColor',rgb('Blue'),'MarkerEdgeColor',rgb('Blue')); % Blue
scatter(cam_58_ped15(:,9), cam_58_ped15(:,10),4,'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple')); % Purple
figure
hold on
scatter(cam_57_ped52(1:2,9), cam_57_ped52(1:2,10),'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red')); % Red
scatter(cam_58_ped52(1:2,9), cam_58_ped52(1:2,10),'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange')); % Orange
scatter(cam_57_ped15(1:2,9), cam_57_ped15(1:2,10),'MarkerFaceColor',rgb('Blue'),'MarkerEdgeColor',rgb('Blue')); % Blue
scatter(cam_58_ped15(1:2,9), cam_58_ped15(1:2,10),'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple')); % Purple

% Compute scores
previous_test_targets = [cam_57_ped52(1,:);cam_58_ped52(1,:);cam_57_ped15(1,:);cam_58_ped15(1,:)];
test_targets = [cam_57_ped52(2,:);cam_58_ped52(2,:);cam_57_ped15(2,:);cam_58_ped15(2,:)];
Scores = zeros(4,4);
NOT_POSSIBLE = 0;

for a=1:size(Scores,1)
    for b=1:size(Scores,2)
        if a == b % Same target
            Scores(a,b) = NOT_POSSIBLE;
        else
            % Belonging to the same camera
            if test_targets(a,1) == test_targets(b,1)
                Scores(a,b) = NOT_POSSIBLE;
            else
                % Compute score
                targ1 = test_targets(a,:); targ2 = test_targets(b,:);
                prev_targ1 = previous_test_targets(a,:); prev_targ2 = previous_test_targets(b,:);
                p1 = targ1(9:10);
                p2 = targ2(9:10);
                pos_score = abs(p1 - p2);
                v1 = (p1 - prev_targ1(9:10))/(1/fps);
                v2 = (p2 - prev_targ2(9:10))/(1/fps);
                vel_score = abs(v1 - v2);
                img1 = imcrop(cameraListImages{1}{targ1(2)},targ1(3:6));
                I1 = [imhist(img1(:,:,1)) imhist(img1(:,:,2)) imhist(img1(:,:,3))];
                img2 = imcrop(cameraListImages{2}{targ2(2)},targ1(3:6));

                I2 = [imhist(img2(:,:,1)) imhist(img2(:,:,2)) imhist(img2(:,:,3))];
                appearance_score = abs(I1 - I2);

                % Inverse since the algorithm tries to bind the maximum
                Scores(a,b) = 1/(sum(pos_score) + sum(vel_score) + (1/3)*sum(appearance_score(:,1),1)/(256*max(appearance_score(:,1))) + (1/3)*sum(appearance_score(:,2),1)/(256*max(appearance_score(:,2))) + (1/3)* sum(appearance_score(:,3),1)/(256*max(appearance_score(:,3))));

            end
        end
    end
end
% Compile mex if not already compiled
if ~exist('utils/fastAuction_v2.6/auctionAlgorithmSparseMex.mexa64', 'file')
    mex -largeArrayDims auctionAlgorithmSparseMex.cpp -lut
end
Scores = sparse(Scores);
% scale A such that round(Ascaled) has sufficient accuracy
scalingFactor = 10^6;
Scores_scaled = Scores * scalingFactor;

% solve assignment problem
tic
[assignments, P] = sparseAssignmentProblemAuctionAlgorithm(Scores_scaled);
time = toc;
%%===========================================
% After the best assignments, recompute homography matrix
% Take the mean of the assigned points, this will be the input to the computation
mean_points = zeros(size(test_targets,1),2+size(test_targets,2));
for i=1:size(mean_points,1)
    couple = [test_targets(i,:); test_targets(assignments(i),:)];
    mean_points(i,:) = [mean(couple(:,9:10)) test_targets(i,:)];
    plot(mean_points(i,1),mean_points(i,2),'ko');
end
% Using the mean points and the original camera space points calculate both homographies
new_Hs = cell(length(cameras),1);
for id=1:length(cameras)
    pin = cell(size(regions_xi,1)+size(mean_points,1),1);
    pout = cell(size(regions_xi,1)+size(mean_points,1),1);
    % Get the regions, we want these to be mapped as well
    regions_xi = outlier_removal_regions{id}(:,1);
    regions_yi = outlier_removal_regions{id}(:,2);
    regions_xi_ = ground_plane_regions{id}(:,1);
    regions_yi_ = ground_plane_regions{id}(:,2);
    for r=1:size(regions_xi,1)
        pin{r} = [regions_xi(r) regions_yi(r)]; % xi,yi
        pout{r} = [regions_xi_(r) regions_yi_(r)]; % xi_, yi_
    end
    % TODO is this necessary?
    r = size(regions_xi,1);
    % Get the mean_points for this camera
    for n=1:size(mean_points,1)
        if mean_points(i,3) == cameras{id}
            xi = mean_points(n,5)+mean_points(7)/2;
            yi = mean_points(n,6)+mean_points(8);
            xi_ = mean_points(n,1);
            yi_ = mean_points(n,2);
            pin{n+r} = [xi yi];
            pout{n+r} = [xi_ yi_];
        end
    end
    pin = cell2mat(pin');
    pout = cell2mat(pout');
    new_Hs{id} = homography_solve(pin',pout');
end
new_estimate1 = new_Hs{1}*[test_targets(1,3)+test_targets(1,5)/2; test_targets(1,4)+test_targets(1,6); 1];
new_estimate2 = new_Hs{2}*[test_targets(2,3)+test_targets(2,5)/2; test_targets(2,4)+test_targets(2,6); 1];
new_estimate1 = new_estimate1./new_estimate1(3);
new_estimate2 = new_estimate2./new_estimate2(3);
plot(new_estimate1(1),new_estimate1(2),'g*');
plot(new_estimate2(1),new_estimate2(2),'g*');

%%=========================================================
cam_id = 1;
k = g_candidates^2;

for f = start_frame:(start_frame+frame_number)
    %-------------This is done for every frame
    disp(['Tracking in frame:  ' sprintf('%d',f) '  @  camera:  ' sprintf('%d',cameras{cam_id}) '...']);
    % Number of targets in this frame
    n = size(allDetections{cam_id}{f},1); %j

    % Appearance cues
    [c_a, allbbs] = appearanceConstraint(n,k,f,allDetections,cameraListImages,lambda,'naive','hda');

end
