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
fprintf('Getting detections from ACF...\n');
allDetections = ACFdetect();
nofilter = allDetections;
% In allDetections get the first ground_truth pedestrian id that is not 999 and filter using the occlusion bit
allDetections = filterACFInria(allDetections); % If you want more than 1 ground_truth label for each ped you can change it here
%%=========================================================
% Plot detections of all peds in both cameras (we consider that peds are represented by the bottom middle of their BB's)
%plotDetectionsCameraSpace(allDetections,cameraListImages,'hda');
%%=========================================================
fprintf('Loading homographies and camera regions...\n');
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
fprintf('Computing camera regions (and overlaps) + detections in the ground plane....\n');
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
fprintf('Grouping detections and ground_truth in frames for both cameras...\n');
% Group allDetections and ground_truth in terms of frames for both cameras
ground_truth = cell(length(cameras),1);
for i=1:length(cameras)
    ground_truth{i} = accumarray(allDetections{i}(:,8),(1:size(allDetections{i},1)).',[],@(x){allDetections{i}(x,:)},{});
end
for i=1:length(cameras)
    allDetections{i} = accumarray(allDetections{i}(:,2),(1:size(allDetections{i},1)).',[],@(x){allDetections{i}(x,:)},{});
end
%%=========================================================
fprintf('Loading raw images...\n');
% Get camera images
% NOTE These two following values, especially sample size are likely to be altered for the actual tracking problem for debug reasons
sample_size = 220; % Number of frames
start_frame = 1150; % Frames to start collecting images
cameraListImages = loadImages(cameras, image_directory, sample_size, start_frame, 'hda');

%%=========================================================
% Plot 4 sample_size images to show detections (with bounding boxes)
%plotDebugBoundingBoxes(cameraListImages,allDetections,start_frame,'hda');
% %%=======================================================
fprintf('Loading ground truth...\n');
% Get a ground truth for an experiment, use RunToVisualize to see these
allPedIds = ground_truth;
allPedIds = computeAllPedIds(homographies, allPedIds, cameras);
ped22 = {allPedIds{1}{22}(145:166,:); allPedIds{2}{22}(73:90,:)};
ped24 = {allPedIds{1}{24}(77:79,:); allPedIds{2}{24}(42:44,:)};
%%=========================================================
fprintf('Starting tracking loop:\n');
% Global iteration loop
k = g_candidates ^ 2; % Candidates per target
start_frames = [1336 1200];
num_frames = [30 30];
%---------------------------------------------------------------------------
for f = 1:(num_frames-1)
    fprintf(['Frame ' num2str(f) ':\n']);
    %---------------------------------------------------------------------------
    fprintf('\t Getting targets and candidates...\n');
    n = 0; % Number of targets in all cameras
    k_total = 0; % Number of candidates in all cameras
    targs = {}; % Actual targets from both cameras
    cands = {}; % Candidates from both cameras
    for c = 1:length(cameras)
        a = allDetections{c}{start_frames(c)+f};
        b = allDetections{c}{start_frames(c)+(f+1)}; % Get the next frame and its detections (i.e our candidates) for association with our targets
        targs{c} = zeros(size(a,1),10); cands{c} = zeros(size(b,1),10);
        % Map targets at frame f or f+1 to the ground plane using homographies
        for i=1:size(targs{c},1)
            targ = a(i,:);
            t = H(homographies{c},[targ(3)+targ(5)/2; targ(4)+targ(6)]);
            targs{c}(i,:) = [targ t(1) t(2)];
        end
        for j=1:size(cands{c},1)
            cand = b(j,:);
            t = H(homographies{c},[cand(3)+cand(5)/2; cand(4)+cand(6)]);
            cands{c}(j,:) = [cand t(1) t(2)];
        end
    end
    n1 = size(targs{1},1); n2 = size(targs{2},1); n = n1 + n2;
    k1 = size(cands{1},1); k2 = size(cands{2},1); k_total = k1 + k2; %k is already defined
    %---------------------------------------------------------------------------
    fprintf('\t Solving assignment of targets...\n');
    %% Pedestrian/Camera association (we do assymetric target association)
    assignmentAlgorithm = 'jonker_volgenant';
    S = zeros(n1,n2); images = {cameraListImages{1}{start_frames(1)+f},cameraListImages{2}{start_frames(2)+f}};
    [assignments, S] = solve_assignment_v2(S, images, targs, assignmentAlgorithm)
    %---------------------------------------------------------------------------

    fprintf('\t Computing appearance cues...\n');
    % Compute local cues - appearance and motion
    c_as = {};
    for c = 1:length(cameras)
      % c_a is coded row-wise that is x=0 y=0 y=1 y=2 y=3 y=4, x=1 y=0 etc...
      [c_a, c_m, c_nm] = deal(zeros(n,1));
      % Show the images for debug
      figure; hold on;
      % Draw the targets
      subplot(3,1,1), subimage(cameraListImages{c}{start_frames(c)+f});
      drawBBs(targs{c}(:,3:6), 'Yellow', 'hda');
      title(['Targets in camera ' num2str(cameras{c})]);
      % Draw the candidates
      subplot(3,1,2), subimage(cameraListImages{c}{start_frames(c)+f+1});
      title(['Candidates in camera ' num2str(cameras{c})]);
      drawCandidateBBs(cands{c}(:,3:6), 'Green', 'hda', k);
      % Appearance cues computed locally
      [c_a, weights, Z, y] = appearanceConstraint_v2(k,size(cands{c},1),cands{c},cameraListImages{c}{start_frames(c)+f+1},'naive',lambda);
      c_as{c} = c_a;

      % NOTE: Debug
      [~,m] = min(c_a)
      x_c = fix(m/5)
      y_c = m-1 - (x_c) * 5
      cx = cands{c}(1,3) + (x_c-2) * cands{c}(1,5)/10;
      cy = cands{c}(1,4) + (y_c-2) * cands{c}(1,6)/10;
      hold on
      rectangle('Position',[cx cy cands{c}(1,5:6)],'EdgeColor', 'Magenta', 'LineWidth', 1);

      subplot(3,1,3), bar(1:25,c_a);
      xticks(1:25);

    end
    kill
    %---------------------------------------------------------------------------
    % Compute global cues - grouping (we ignore spatial proximity cues)
    Cg = groupConstraint_v2(n);

    %---------------------------------------------------------------------------
    fprintf('\tSolving Frank-Wolfe optimization...\n');
    % Prepare inputs for Frank Wolfe (conditional gradient)
    [A,b,Aeq,Beq,labels] = FW_preamble(n,k,c_a,c_m,c_nm,Cg);
    % Solve the problem using Frank Wolfe
    [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(A,b, Aeq, Beq, labels); % minx is the value we want
    % Get chunk of k candidates for target i and see which one was picked
    optimization_results = reshape(minx,k,[]);
    %NOTE: Two targets may be assigned to the same target
    fprintf('\tFound optimal candidates for each target. Storing the tracks...\n');
    % TODO store the tracks
    %---------------------------------------------------------------------------
    % Use target association info to fix tracks and fix homography (maybe fix homography only every other frame)



    %---------------------------------------------------------------------------
end
