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
%camera_plane_regions{1} = [59.7037 796.9403; 503.3571 353.2869; 1015.9567 375.7695; 1011.4602 798.4391];
%camera_plane_regions{2} = [4.2471 796.4391; 5.7459 491.1792; 908.0410 399.7506; 1098.3923 798.4391];
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
    % Plot pedestrians
    % Draw regions
    colors = {'Red','Green'};
    for i=1:length(cameras)
        plotDetectionsGroundPlane(allDetections,homographies, ground_plane_regions, 'show_outliers', 'hda');
        drawPoly(ground_plane_regions{i},colors{i},0.5,false);
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
%draw_elevator_patio(gplane);

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
% TODO We have 4 targets so score is 4x4, generalize
Scores = zeros(4,4);
assignmentAlgo = 'jonker_volgenant';
[assignments, P] = solve_assignment(Scores, test_targets, previous_test_targets, cameraListImages, assignmentAlgo);

%%===========================================
% After the best assignments, recompute homography matrix
% Take the mean of the assigned points, this will be the input to the computation
mean_points = zeros(size(test_targets,1),2+size(test_targets,2));
for i=1:size(mean_points,1)
    couple = [test_targets(i,:); test_targets(assignments(i),:)];
    mean_points(i,:) = [mean(couple(:,9:10)) test_targets(i,:)];
end
plot(mean_points(:,1),mean_points(:,2),'ko');
% Using the mean points and the original camera space points calculate both homographies
new_Hs = cell(length(cameras),1);
rho_r = 3;
rho_m = 5;
for id=1:length(cameras)
    % --------------------------------------------------------------------------
    pin = {};
    pout = {};
    % Get the mean_points for this camera
    for reps = 1:rho_m
        for n=1:size(mean_points,1)
            if mean_points(n,3) == cameras{id}
                xi = mean_points(n,5) + mean_points(n,7)/2;
                yi = mean_points(n,6) + mean_points(n,8);
                xi_ = mean_points(n,1);
                yi_ = mean_points(n,2);
                pin{end+1} = [xi yi];
                pout{end+1} = [xi_ yi_];
            end
        end
    end
    % --------------------------------------------------------------------------
    % Get the regions, we want these to be mapped as well
    regions_xi = outlier_removal_regions{id}(:,1);
    regions_yi = outlier_removal_regions{id}(:,2);
    regions_xi_ = ground_plane_regions{id}(:,1);
    regions_yi_ = ground_plane_regions{id}(:,2);
    % NOTE Additional repeated points, A. Bernardino suggestion
    for reps = 1:rho_r
        for r=1:size(regions_xi,1)
            pin{end+1} = [regions_xi(r) regions_yi(r)]; % xi,yi
            pout{end+1} = [regions_xi_(r) regions_yi_(r)]; % xi_, yi_
        end
    end

    pin = cell2mat(pin');
    pout = cell2mat(pout');
    tic
    new_Hs{id} = solve_homography(pin',pout','svd');
    homography_time = toc;
    disp(strcat('Homography computation takes: ', num2str(homography_time)));
end


% Plot new point estimates
new_estimate1 = new_Hs{1} * [test_targets(3,3)+test_targets(3,5)/2; test_targets(3,4)+test_targets(3,6); 1];
new_estimate2 = new_Hs{2} * [test_targets(4,3)+test_targets(4,5)/2; test_targets(4,4)+test_targets(4,6); 1];
new_estimate1 = new_estimate1./new_estimate1(3);
new_estimate2 = new_estimate2./new_estimate2(3);
plot(new_estimate1(1),new_estimate1(2),'g*');
plot(new_estimate2(1),new_estimate2(2),'m*');
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

%%=========================================================
k = g_candidates^2; % Candidates per target
start_frames = [1139 1030]; num_frames = [2 2]; % NOTE DEBUG FOR NOW
command;
%---------------------------------------------------------------------------
for f = start_frames(id):(start_frames(id) + (num_frames(id)-1))
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
