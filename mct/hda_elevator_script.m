% Set params of the capture
setCaptureParams_hda_elevator;
% Set params of the calibration
setDetectionParams_hda_elevator;
% Set parameters for the tracker
warning off;
setTrackerParams;
% TODO: eliminating warnings of newline inbuilt function with this
warning on;
% Pick if you want to display debug images or not
show_images = 'on';
debug_plots = false;
set(0,'DefaultFigureVisible',show_images);
%%=========================================================
allDetections = ACFdetect();
% In allDetections get the first ground_truth pedestrian id that is not 999 and filter using the occlusion bit
allDetections = filterACFInria(allDetections); % If you want more than 1 ground_truth label for each ped you can change it here
%%=========================================================
% Plot detections of all peds in both cameras (we consider that peds are represented by the middle of their BB's)
%plotDetectionsCameraSpace(allDetections,cameraListImages,'hda');
%%=========================================================
% Load homographies, these homographies where obtained via the use of cp2tform
[homographies, invhomographies] = loadHomographies(homography_directory,'hda'); % Defined in global variables
% Plot regions -- we try to be as permissive as possible in these regions while keeping them simple
% Alternatively we were provided with very detailed regions from the people who own the datasets, we can use both
outlier_removal_regions = cell(length(cameras),1);
for i=1:length(cameras)
    outlier_removal_regions{i} = load(strcat(visibility_regions_directory,num2str(cameras{i}),'.mat'));
    outlier_removal_regions{i} = outlier_removal_regions{i}.t;
end
%%=========================================================
ground_plane_regions = computeGroundPlaneRegions(outlier_removal_regions, homographies, length(cameras), 'hda');
if debug_plots == true
    openfig(floor_image); hold on;
    colors = {'Red','Green'};
    for i=1:length(cameras)
        plotDetectionsGroundPlane(allDetections,homographies, ground_plane_regions, 'show_outliers', 'hda'); % Plot pedestrians
        drawPoly(ground_plane_regions{i},colors{i},0.5,false); % Draw regions
    end
end
% Plot overlap of camera regions
[overlap, ~, ~] = computeOverlap(ground_plane_regions); % Performed in the pixel plane
if debug_plots == true
    drawPoly(overlap,'Black',1.0,false);
end
%%=========================================================
% Group allDetections and ground_truth in terms of frames for both cameras
ground_truth = cell(length(cameras),1);
for i=1:length(cameras)
    ground_truth{i} = accumarray(allDetections{i}(:,8),(1:size(allDetections{i},1)).',[],@(x){allDetections{i}(x,:)},{});
end
for i=1:length(cameras)
    allDetections{i} = accumarray(allDetections{i}(:,2),(1:size(allDetections{i},1)).',[],@(x){allDetections{i}(x,:)},{});
end
%%=========================================================
% Get camera images
% NOTE These two following values, especially sample size are likely to be altered for the actual tracking problem for debug reasons
sample_size = 500; % Number of frames
start_frame = 1000; % Frames to start collecting images
cameraListImages = loadImages(cameras, image_directory, sample_size, start_frame);
%%=========================================================
% Plot 4 sample_size images to show detections (with bounding boxes)
%plotDebugBoundingBoxes(cameraListImages,allDetections,start_frame,'hda');
%%=========================================================
% Build POM (Fleuret et. al)
% gplane = imread(floor_image);
% gplane = imcrop(gplane,elevator_patio);
% figure
% new_plane = draw_elevator_patio(gplane);
% hold on;
% drawGrid(new_plane,0.5,'hda');
% set(gca,'ydir','normal');
% %%=========================================================
% allPedIds = ground_truth;
% %%=========================================================
% % Get a ground truth for an experiment
% allPedIds = computeAllPedIds(homographies, allPedIds, cameras);
% cam57 = 1; cam58 = 2;
% ped1 = 52; ped2 = 15;
% cam_57_ped52 = allPedIds{cam57}{ped1}(5:137,:);
% cam_58_ped52 = allPedIds{cam58}{ped1}(6:108,:);
% scatter(cam_57_ped52(:,9), cam_57_ped52(:,10),4,'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red')); % Red
% scatter(cam_58_ped52(:,9), cam_58_ped52(:,10),4,'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange')); % Orange
%
% cam_57_ped15 = allPedIds{cam57}{ped2}(26:92,:);
% cam_58_ped15 = allPedIds{cam58}{ped2}(28:86,:);
% scatter(cam_57_ped15(:,9), cam_57_ped15(:,10),4,'MarkerFaceColor',rgb('Blue'),'MarkerEdgeColor',rgb('Blue')); % Blue
% scatter(cam_58_ped15(:,9), cam_58_ped15(:,10),4,'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple')); % Purple

%%=========================================================
% Global iteration loop
k = g_candidates ^ 2; % Candidates per target
start_frames = [1139 1030];
num_frames = [2 2];
%---------------------------------------------------------------------------
for f = start_frames(id):(start_frames(id) + (num_frames(id)-1))

    % Map detections at f to the ground plane using homographies

    % Solve target coupling in the overlap zones
    %---------------------------------------------------------------------------
    n = 0; % All targets in all cameras
    for id = 1:length(cameras)
        n = n + size(allDetections{id}{f},1);
    end
    %---------------------------------------------------------------------------
    % Create global arrays and matrices
    [c_a, c_m, c_nm] = deal(zeros(n,1));
    Cg = zeros(n,n)
    %---------------------------------------------------------------------------
    % Compute global Csp

    % Compute global Cg

    %---------------------------------------------------------------------------
    %for id = 1:length(cameras)
    for id = 1:1
        % NOTE: l prefix means local
        disp(['Tracking in frame:  ' sprintf('%d',f) '  @  camera:  ' sprintf('%d',cameras{id}) '...']);
        l_n = size(allDetections{id}{f},1);
        disp(['Number of targets in this frame for this camera: ' sprintf('%d', l_n)]);
        prev_n = 0;
        if id ~= 1
            prev_n = size(allDetections{id-1}{f},1);
        end

        % Appearance cues computed locally
        [l_c_a, allbbs, allbb_imgs] = appearanceConstraint(l_n,k,f,allDetections,cameraListImages,lambda,'naive','hda',id);
        c_a(prev_n * k + 1:(prev_n * k + 1) + l_n * k) = l_c_a;

        % Motion Constraint
        %l_c_m = motionConstraint(l_n,k,f,fps,allDetections,predictions,past_observations);

        % Neighbourhood motion Constraint - all targets are neighbours since we are not in a crowded scenery
        %l_c_nm = neighbourhoodMotionConstraint(l_n,k,f,fps,allDetections,predictions,past_observations);
    end
    %---------------------------------------------------------------------------
    [A,b,Aeq,Beq,labels] = FW_preamble(n,k,c_a,c_m,c_nm,Csp,Cg);

    % Solve the problem
    [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(A,b, Aeq, Beq, labels); % minx is the value we want

    % Get chunk of k candidates for target i and see which one was picked
    optimization_results = reshape(minx,k,[]);

    %---------------------------------------------------------------------------


end

figure
testped = 2;
for i=1:k
    subplot(sqrt(k),sqrt(k),i)
    imshow(allbb_imgs{testped}{i})

end
