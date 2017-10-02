% Set params of the capture
setCaptureParams_hda_elevator;
% Set params of the calibration
setDetectionParams_hda_elevator;
% Pick if you want to display debug images or not
show_images = 'on';
set(0,'DefaultFigureVisible',show_images);
%%=========================================================
allDetections = ACFdetect();
% In allDetections get the first ground_truth pedestrian id that is not 999 and filter using the occlusion bit
allDetections = filterACFInria(allDetections); % If you want more than 1 ground_truth label for each ped you can change it here
%%=========================================================
% Plot 4 sample_size images to show detections (with bounding boxes)
start_frame = 17;
%plotDebugBoundingBoxes_hda('~/HDA_Dataset_V1.3/hda_image_sequences_matlab/',allDetections,start_frame);
%%=========================================================
% Plot detections of all peds in both cameras (we consider that peds are represented by the middle of their BB's)
%plotDetectionsCameraSpace_hda(allDetections);
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
%openfig('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/7th_floor_ground_plane_reference_frame_map.fig');
%hold on;
ground_plane_regions = computeHomoplanes(outlier_removal_regions, homographies, length(cameras), 'hda');
% Plot pedestrians
%plotDetectionsGroundPlane_hda(allDetections,homographies, ground_plane_regions, 'show_outliers');
% Draw regions
colors = {'Red','Green'};
for i=1:length(cameras)
    %drawPoly(ground_plane_regions{i},colors{i},0.5,false);
end
% Plot overlap of camera regions
[overlap, kmat, mmat] = computeOverlap(ground_plane_regions); % Performed in the pixel plane
%drawPoly(overlap,'Black',1.0,false);
%%=========================================================
% Group allDetections in terms of frames for both cameras
for i=1:length(cameras)
    allDetections{i} = accumarray(allDetections{i}(:,2),(1:size(allDetections{i},1)).',[],@(x){allDetections{i}(x,:)},{});
end
%%=========================================================
% Get camera images
data_folders = '~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/cam';
sample_size = 500;
for i=1:length(cameras)
    if exist(data_folders(i),'dir')
        foldercontent = dir(strcat(data_folders,num2str(cameras{i})));
        if numel(foldercontent) == 2
            %# folder exists and is empty
            seq2img(cameras{i},start_frame, sample_size);
        end
    end
    cameraListImages{i} = {};
    for j=1:sample_size
        cameraListImages{i}{j} = imread(strcat(data_folders, num2str(cameras{i}), '/', num2str(j), '.png'));
    end
end
%%=========================================================
% Build POM
ref_img = imread('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/ref_58.png');
test_img = imread('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/test_img_58.png');
test_frame = 450;
[img_diff, bin_img] = convertBB2bin_img(allDetections{2}{test_frame}, test_img, ref_img);
%figure
%imshow(out);
figure
imshow(bin_img);
hold off
figure
gplane = imread('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/7th_floor_ground_plane_reference_frame_map.png');
gplane = imcrop(gplane,[198,167,100,100]);
imshow(gplane);
hold on;
M = size(gplane,1); N = size(gplane,2);
grid_granularity = 10;
for k = 1:grid_granularity:M
    x = [1 N]; y = [k k];
    plot(x,y,'Color','w','LineStyle','-'); plot(x,y,'Color','k','LineStyle',':');
end
for k = 1:grid_granularity:N
    x = [k k]; y = [1 M];
    plot(x,y,'Color','w','LineStyle','-'); plot(x,y,'Color','k','LineStyle',':');
end


% Draw a ground truth for an experiment


%%=========================================================
cam_id = 1;
frame_number = 10; %Number of frames
start_frame = 70;
g = 5; k = g^2; % k = 25
lambda = 0.1; % variable for the appearance cues
zeta = 0.3; % weight of the motion constraint
eta = 0.2; % weight of the neighbourhood motion constraint

for f = start_frame:(start_frame+frame_number)
    %-------------This is done for every frame
    disp(['Tracking in frame:  ' sprintf('%d',f) '  @  camera:  ' sprintf('%d',cameras{cam_id}) '...']);
    % Number of targets in this frame
    n = size(allDetections{cam_id}{f},1); %j

    % Appearance cues
    c_a = zeros(n*k,1);
    %[c_a, allbbs] = appearanceConstraint(n,k,f,allDetections,cameraListImages,lambda,'naive','hda');


end
