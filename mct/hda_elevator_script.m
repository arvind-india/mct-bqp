% Set params of the capture
setCaptureParams_hda_elevator;
% Set params of the calibration
setDetectionParams_hda_elevator;
% Set parameters for the tracker
setTrackerParams;
% Pick if you want to display debug images or not
show_images = 'on'; debug_plots = false;
set(0,'DefaultFigureVisible',show_images);
%%=========================================================
allDetections = ACFdetect();
% In allDetections get the first ground_truth pedestrian id that is not 999 and filter using the occlusion bit
allDetections = filterACFInria(allDetections); % If you want more than 1 ground_truth label for each ped you can change it here
%%=========================================================
% Plot detections of all peds in both cameras (we consider that peds are represented by the bottom middle of their BB's)
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
cameraListImages = loadImages(cameras, image_directory, sample_size, start_frame, 'hda');

%%=========================================================
% Plot 4 sample_size images to show detections (with bounding boxes)
%plotDebugBoundingBoxes(cameraListImages,allDetections,start_frame,'hda');
% %%=======================================================
% Get a ground truth for an experiment, use RunToVisualize to see these
allPedIds = ground_truth;
allPedIds = computeAllPedIds(homographies, allPedIds, cameras);
ped22 = {allPedIds{1}{22}(145:166,:); allPedIds{2}{22}(73:90,:)};
ped24 = {allPedIds{1}{24}(77:79,:); allPedIds{2}{24}(42:44,:)}
%%=========================================================
% Global iteration loop
k = g_candidates ^ 2; % Candidates per target
start_frames = [1336 1200];
num_frames = [30 30];
%---------------------------------------------------------------------------
for f = 1:(num_frames-1)
    %---------------------------------------------------------------------------
    n = 0; % Number of targets in all cameras
    targs = {} % Actual targets from both cameras
    for c = 1:length(cameras)
        n = n + size(allDetections{c}{start_frames(c)+f},1);
        targs{c} = allDetections{c}{start_frames(c)+f};
        % Map targets at frame f to the ground plane using homographies
        for i=1:size(targs{c},1)
            targ = targs{c};
            t = H(homographies{c},[targ(3)+targ(5)/2; targ(4)+targ(6)]);
            targs{c} = [targ(1) targ(2) t(1) t(2) targ(8)];
        end
    end
    n1 = size(targs{1},1); n2 = size(targs{2},1);
    %---------------------------------------------------------------------------
    %% Pedestrian/Camera association (we do assymetric target association)
    % TODO
    %Scores = zeros(4,4);
    %assignmentAlgo = 'jonker_volgenant';
    %[assignments, P] = solve_assignment(Scores, test_targets, previous_test_targets, cameraListImages, assignmentAlgo);
    %---------------------------------------------------------------------------
    %% Frame association
    % Get the next frame and its detections (i.e our candidates) for association with our targets
    k = 0; % Number of candidates in all cameras
    cands = {} % Candidates from both cameras
    for c = 1:length(cameras)
        n = n + size(allDetections{c}{start_frames(c)+(f+1)},1);
        cands{c} = allDetections{c}{start_frames(c)+(f+1)};
        % Map candidates at frame f to the ground plane using homographies
        for j=1:size(cands{c},1)
            cand = cands{c};
            t = H(homographies{c},[cand(3)+cand(5)/2; cand(4)+cand(6)]);
            cands{c} = [cand(1) cand(2) t(1) t(2) cand(8)];
        end
    end
    k1 = size(cands{1},1); k2 = size(cands{2},1);
    %---------------------------------------------------------------------------
    % Create global arrays and matrices
    [c_a, c_m, c_nm] = deal(zeros(n,1)); Cg = zeros(n,n);

    %---------------------------------------------------------------------------
    % Compute local cues - motion and appearance
    for c = 1:length(cameras)
          % Appearance cues computed locally
    %     [c_a, allbbs, allbb_imgs] = appearanceConstraint(l_n,k,f,allDetections,cameraListImages,lambda,'naive','hda',id);
    %     c_a(prev_n * k + 1:(prev_n * k + 1) + l_n * k) = l_c_a;
    %
    %     % Motion Constraint
    %     %l_c_m = motionConstraint(l_n,k,f,fps,allDetections,predictions,past_observations);
    %
    %     % Neighbourhood motion Constraint - all targets are neighbours since we are not in a crowded scenery
    %     %l_c_nm = neighbourhoodMotionConstraint(l_n,k,f,fps,allDetections,predictions,past_observations);
    end

    %---------------------------------------------------------------------------
    % Compute global cues - grouping (we ignore spatial proximity cues)


    %---------------------------------------------------------------------------
    % Prepare inputs for Frank Wolfe
    [A,b,Aeq,Beq,labels] = FW_preamble(n,k,c_a,c_m,c_nm,Cg);
    % Solve the problem using Frank Wolfe
    [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(A,b, Aeq, Beq, labels); % minx is the value we want
    % Get chunk of k candidates for target i and see which one was picked
    optimization_results = reshape(minx,k,[]);

    kill
    %---------------------------------------------------------------------------
    % Use target association info to fix tracks and fix homography
    
    %---------------------------------------------------------------------------
end

% figure
% testped = 2;
% for i=1:k
%     subplot(sqrt(k),sqrt(k),i)
%     imshow(allbb_imgs{testped}{i})
% end
