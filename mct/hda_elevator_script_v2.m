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
cameraListImages = cell(length(cameras),1);
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
    openfig(floor_image);
    hold on;
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
for i=1:length(cameras)
    direct = strcat(image_directory, num2str(cameras{i}));
    if exist(direct{1},'dir')
        foldercontent = dir(direct{1});
        if numel(foldercontent) == 2
            % folder exists and is empty
            seq2img(cameras{i},start_frame, sample_size);
        end
    end
    %cameraListImages{i} = cell(sample_size,1);
    for j=start_frame:(start_frame + sample_size)
        cameraListImages{i}{j} = imread(strcat(direct{1}, '/', num2str(j), '.png'));
    end
end
%%=========================================================
% Plot 4 sample_size images to show detections (with bounding boxes)
%plotDebugBoundingBoxes(cameraListImages,allDetections,start_frame,'hda');
%%=========================================================
% Build POM


%%=========================================================
gplane = imread(floor_image);
gplane = imcrop(gplane,elevator_patio);
figure
new_plane = draw_elevator_patio(gplane);
hold on;
drawGrid(new_plane,0.5,'hda');
set(gca,'ydir','normal');
%%=========================================================
allPedIds = ground_truth;
%%=========================================================
% Get a ground truth for an experiment
allPedIds = computeAllPedIds(homographies, allPedIds, cameras);
cam57 = 1; cam58 = 2;
ped1 = 52; ped2 = 15;
cam_57_ped52 = allPedIds{cam57}{ped1}(5:137,:);
cam_58_ped52 = allPedIds{cam58}{ped1}(6:108,:);
scatter(cam_57_ped52(:,9), cam_57_ped52(:,10),4,'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red')); % Red
scatter(cam_58_ped52(:,9), cam_58_ped52(:,10),4,'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange')); % Orange

cam_57_ped15 = allPedIds{cam57}{ped2}(26:92,:);
cam_58_ped15 = allPedIds{cam58}{ped2}(28:86,:);
scatter(cam_57_ped15(:,9), cam_57_ped15(:,10),4,'MarkerFaceColor',rgb('Blue'),'MarkerEdgeColor',rgb('Blue')); % Blue
scatter(cam_58_ped15(:,9), cam_58_ped15(:,10),4,'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple')); % Purple

figure; hold on; %draw_elevator_patio(gplane);

% scatter(cam_57_ped52(1:2,9), cam_57_ped52(1:2,10),'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red')); % Red
% scatter(cam_58_ped52(1:2,9), cam_58_ped52(1:2,10),'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange')); % Orange
% scatter(cam_57_ped15(1:2,9), cam_57_ped15(1:2,10),'MarkerFaceColor',rgb('Blue'),'MarkerEdgeColor',rgb('Blue')); % Blue
% scatter(cam_58_ped15(1:2,9), cam_58_ped15(1:2,10),'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple')); % Purple
scatter(cam_57_ped15(2,9), cam_57_ped15(2,10),'MarkerFaceColor',rgb('Blue'),'MarkerEdgeColor',rgb('Blue')); % Blue
scatter(cam_58_ped15(2,9), cam_58_ped15(2,10),'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple')); % Purple
scatter(cam_57_ped52(2,9), cam_57_ped52(2,10),'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red')); % Red
scatter(cam_58_ped52(2,9), cam_58_ped52(2,10),'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange')); % Orange
% Compute scores
previous_test_targets = [cam_57_ped52(1,:);cam_58_ped52(1,:);cam_57_ped15(1,:);cam_58_ped15(1,:)];
test_targets = [cam_57_ped52(2,:);cam_58_ped52(2,:);cam_57_ped15(2,:);cam_58_ped15(2,:)];
% TODO We have 4 detections (2 targets) so score is 4x4, generalize
Scores = zeros(4,4);
assignmentAlgo = 'jonker_volgenant';
[assignments, P] = solve_assignment(Scores, test_targets, previous_test_targets, cameraListImages, assignmentAlgo);

%%==============================================================================
% After the best assignments, recompute homography matrix
% Take the mean of the assigned points, this will be the input to the computation
pedestrian_points = zeros(size(test_targets,1),4 + size(test_targets,2));
for i=1:size(pedestrian_points,1)
    couple = [test_targets(i,:); test_targets(assignments(i),:)];
    pedestrian_points(i,:) = [couple(1,9:10) couple(2,9:10) test_targets(i,:)];
end
plot(pedestrian_points(:,1),pedestrian_points(:,2),'ko');
% Using the mean points and the original camera space points calculate both homographies
new_Hs = cell(length(cameras),1);
rho_r = 3;
rho_m = 1;
% Method 1 -- fix one of the transformations H1 and correct the other
% Method 2 -- calculate H1 and H2 iteratively, fixing one and then the other
new_Hs{1} = homographies{1};
% --------------------------------------------------------------------------
id = 2;
[pin, pout] = aux_homo_debug(id, cameras, outlier_removal_regions, ground_plane_regions, pedestrian_points, rho_r, rho_m);
tic
new_Hs{id} = solve_homography(pin',pout','svd');
homography_time = toc;
disp(strcat('Homography computation takes: ', num2str(homography_time)));
comparison_second_homo = new_Hs{id};
% --------------------------------------------------------------------------
id = 1;
[pin, pout] = aux_homo_debug(id, cameras, outlier_removal_regions, ground_plane_regions, pedestrian_points, rho_r, rho_m);
tic
new_Hs{id} = solve_homography(pin',pout','svd');
homography_time = toc;
disp(strcat('Homography computation takes: ', num2str(homography_time)));
comparison_first_homo = new_Hs{id};
%--------------------------------------------------------------------------
homography2 = new_Hs{2};
% --------------------------------------------------------------------------

for reps=1:20
    % Recalculate points using the new second Homography
    id = 2;
    [new_Hs, pedestrian_points] = alternating_homog(new_Hs, pedestrian_points, id, outlier_removal_regions, ground_plane_regions, rho_r, rho_m);

    % Recalculate points using the new first Homography
    id = 1;
    [new_Hs, pedestrian_points] = alternating_homog(new_Hs, pedestrian_points, id, outlier_removal_regions, ground_plane_regions, rho_r, rho_m);
    % Plot new point estimates
    new_estimate1 = new_Hs{1} * [test_targets(3,3)+test_targets(3,5)/2; test_targets(3,4)+test_targets(3,6); 1];
    
    new_estimate2 = new_Hs{2} * [test_targets(4,3)+test_targets(4,5)/2; test_targets(4,4)+test_targets(4,6); 1];


    new_estimate1 = new_estimate1./new_estimate1(3);
    new_estimate2 = new_estimate2./new_estimate2(3);
    plot(new_estimate1(1),new_estimate1(2), 'Marker', '*', 'Color',rgb('LightGreen'));
    plot(new_estimate2(1), new_estimate2(2), 'Marker', '*','Color',rgb('LightSalmon'));
end
%%==============================================================================

new_estimate2_before_iteration = comparison_second_homo * [test_targets(4,3)+test_targets(4,5)/2; test_targets(4,4)+test_targets(4,6); 1];
new_estimate1_before_iteration = comparison_first_homo * [test_targets(3,3)+test_targets(3,5)/2; test_targets(3,4)+test_targets(3,6); 1];
new_estimate2_before_iteration = new_estimate2_before_iteration./new_estimate2_before_iteration(3);
new_estimate1_before_iteration = new_estimate1_before_iteration./new_estimate1_before_iteration(3);
plot(new_estimate1_before_iteration(1), new_estimate1_before_iteration(2), 'Marker', '*','Color',rgb('DarkGreen'));
plot(new_estimate2_before_iteration(1), new_estimate2_before_iteration(2), 'Marker', '*', 'Color', rgb('DarkRed'));

plot([new_estimate2_before_iteration(1) new_estimate2(1)],[new_estimate2_before_iteration(2) new_estimate2(2)],'Color',rgb('Red'));
new_estimate2_otherped = new_Hs{2} * [test_targets(2,3)+test_targets(2,5)/2; test_targets(2,4)+test_targets(2,6); 1];
new_estimate2_otherped = new_estimate2_otherped./new_estimate2_otherped(3);
plot(new_estimate2_otherped(1),new_estimate2_otherped(2),'y*');
% Plot new ground_plane_regions estimates
new_reg = cell(length(cameras),1);
colors_new = {'Salmon','Lime'};
colors_orig = {'Red','Green'};
title(strcat('RHO_M: ', num2str(rho_m),',RHO_R: ', num2str(rho_r)));
for i=2:length(cameras) % TODO Doing it only for camera 2 for debug
    new_reg{i} = new_Hs{i} * transpose(horzcat(outlier_removal_regions{i}, ones(size(outlier_removal_regions{i},1),1)));
    new_reg{i}(1,:) = new_reg{i}(1,:)./new_reg{i}(3,:);
    new_reg{i}(2,:) = new_reg{i}(2,:)./new_reg{i}(3,:);
    new_reg{i} = transpose(new_reg{i}(1:2,:));
    drawPoly(new_reg{i}, colors_new{i}, 0.5, false);
    drawPoly(ground_plane_regions{i}, colors_orig{i}, 0.5, false);
end

command;
%%=========================================================
% Global iteration loop
k = g_candidates^2; % Candidates per target
start_frames = [1139 1030]; num_frames = [2 2]; % NOTE DEBUG FOR NOW
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
    [Csp, Cg] = deal(zeros(n));
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
