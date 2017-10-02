% Set params of the capture
setCaptureParams_campus2;
% Set params of the detection
setDetectionParams_campus2;
% Set params of the tracker
setTrackerParams;
% Pick if you want to display debug images or not
show_images = 'on';
set(0,'DefaultFigureVisible',show_images);
%%=========================================================
% Load images
cameraListImages = {};
for i=1:2
    cameraListImages{i} = bbGt('getFiles',{image_directories{i}});
end
% Use the trained Neural Network to do pedestrian detection
sample_size = 200;
allDetections = CNNdetect(cameraListImages, sample_size);
%%=========================================================
% Parse for systematic error detections and remove all empty cells
allDetections = filterCNN_systematicErrors(allDetections);
for i=1:2
    allDetections{i} = allDetections{i}(~cellfun('isempty',allDetections{i}));
end
%%=======================================================
% Plot 4 sample_size images to show detections (with bounding boxes)
start_frame = 4;
plotDebugBoundingBoxes_campus2(cameraListImages,allDetections,start_frame);
%%=======================================================
% Plot detections of all peds in both cameras (we consider that peds are represented by the middle of their BB's)
plotDetectionsCameraSpace_campus2(cameraListImages,allDetections);
%%=======================================================
% Load homographies, these homographies where obtained via the use of cp2tform
[homographies, invhomographies] = loadHomographies(homography_directory,'campus_2'); % Defined in global variables
% Plot regions
inplanes{1} = [494 545; 426 687; 602 681; 852 590; 631 539]; % Alameda cam
inplanes{2} = [162 510; 702 608; 917 558; 603 390; 447 412]; % Central cam
homoplanes = computeHomoplanes(inplanes, homographies, length(cameras), 'campus_2');
% Plot pedestrians
plotDetectionsGroundPlane_campus2(allDetections,homographies);
% Plot overlap of camera regions
overlap = computeOverlap(homoplanes);
%%==============================================================
% Single camera preamble for tracking (uses grouping, appearance, cues, etc...) + uses ILOG CPLEX for Frank-Wolfe optimization
predictions = {}; all_predictions = {}; past_observations = {};
cam_id = 1; %Camera id being considered in this iteration
frame_number = 10; %Number of frames
start_frame = 1;
% Total number of sampled candidates for each target
homography_constraint = false
g = 5; k = g^2; % k = 25
lambda = 0.1; % variable for the appearance cues
zeta = 0.3; % weight of the motion constraint
eta = 0.2; % weight of the neighbourhood motion constraint
%%==============================================================
for f = start_frame:(start_frame + frame_number)
%-------------This is done for every frame
    disp(['Tracking in frame ' sprintf('%d',f) '@ camera ' sprintf('%s',cameras{cam_id}) '...']);
    % Number of targets in this frame
    n = size(allDetections{cam_id}{f},1); %j

    % Appearance cues
    c_a = zeros(n*k,1);
    [c_a, allbbs] = appearanceConstraint(n,k,f,allDetections,cameraListImages,lambda,'naive','campus_2');

    % Spatial Proximity Constraint
    Csp = zeros(n*k);
    Csp = spatialproximityConstraint(n,k,allbbs);

    % Group Constraint
    Cg = zeros(n*k);
    Cg = groupConstraint(n,k,f,allbbs,allDetections);

    % Motion Constraint
    c_m = zeros(n*k,1);
    c_m = motionConstraint(n,k,f,fps,allDetections,predictions,past_observations);

    % Neighbourhood motion Constraint - all targets are neighbours since we are not in a crowded scenery
    c_nm = zeros(n*k,1);
    c_nm = neighbourhoodMotionConstraint(n,k,f,fps,allDetections,predictions,past_observations);

    % Homography Constraint
    if homography_constraint
        omega = 0.1;
        if size(cameraListImages,2) > 1 %Only consider homographies if we have more than 1 camera
            c_h = homographyConstraint(allbbs,allDetections,cam_id,n,k,f,homographies,invhomographies);
        end
    end

    % Prepare the inputs to the Frank Wolfe
    Aeq = zeros(n,k*n);
    for i=1:n
        Aeq(i,(i-1)*k+1:i*k) = ones(k,1);
    end
    Beq = ones(n,1);

    labels = zeros(k*n,1);
    idx = 1;
    for t=1:n
        labels(idx:(idx+k-1)) = ones(k,1)*t;
        idx = idx + k;
    end

    % Model the problem
    A = sparse(Csp + Cg);
    if size(cameraListImages,2) > 1
        %b = c_a + zeta*c_m + eta*c_nm + omega*c_h;
        b = c_a;
    else
        b = c_a + zeta*c_m + eta*c_nm;
    end

    % Solve the problem
    [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(A,b, Aeq, Beq, labels,'swap'); % minx is the value we want

    % Get chunk of k candidates for target i and see which one was picked
    optimization_results = reshape(minx,k,[]);

    past_observations = {}; % Erase past observations
    for target = 1:n
        [r,opt_col] = find(optimization_results(:,target));
        best_candidate_pos = allbbs{target}(r,:);
        % Compute velocity
        target_pos = allDetections{1}{f}(target,:);
        % Store this frame's targets with their speed + pos to be used in motion
        velocity = [((target_pos(3) + target_pos(5)/2) - (best_candidate_pos(1) + best_candidate_pos(3)/2))/(1.0/fps) ((target_pos(4) + target_pos(6)/2) - (best_candidate_pos(2) + best_candidate_pos(4)/2))/(1.0/fps)]
        past_observations{target} = [target_pos(3:4) velocity];
        % Store this frame's targets with their predictions
        predictions{target} = [target_pos(3:6); best_candidate_pos];
    end
    all_predictions{end+1}=predictions;

end

%%================================================================
figure
set(gca, 'XAxisLocation', 'origin'); set(gca, 'YAxisLocation', 'origin');
hold on
for t=1:n
    scatter(past_observations{t}(3),past_observations{t}(4),'DisplayName',['Target ' sprintf('%d',t)]);
end
legend('show');
title('Velocities')
set(gca,'Ydir','reverse');

%%=================================================================
figure
hold on
for f=1:size(all_predictions,2)
    for t=1:size(all_predictions{f},2)
        scatter(all_predictions{f}{t}(1,1)+all_predictions{f}{t}(1,3)./2,all_predictions{f}{t}(1,2)+all_predictions{f}{t}(1,4)./2,'DisplayName',['Target ' sprintf('%d',t)],'MarkerFaceColor',rgb('Blue'),'MarkerEdgeColor',rgb('Blue'));
        scatter(all_predictions{f}{t}(2,1)+all_predictions{f}{t}(2,3)./2,all_predictions{f}{t}(2,2)+all_predictions{f}{t}(2,4)./2,'DisplayName',['Target ' sprintf('%d',t)],'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red'));
    end
end
title('predictions');
axis([0 size(imread(cameraListImages{1}{1}),2) 0 size(imread(cameraListImages{1}{1}),1)])
set(gca,'Ydir','reverse');

%%=================================================================
% if homography_constraint
%     figure
%     subplot(2,2,1);
%     hold on
%     stem(c_h{1});
%     testr = zeros(5,2);
%     startp = 5;
%     for i=1:5
%         testr(i,:) = [startp+(i-1)*5 c_h{1}(startp+(i-1)*5)];
%     end
%     plot(testr(:,1),testr(:,2),'k');
%     hold off
%
%     subplot(2,2,2);
%     stem(c_h{2});
%
%     subplot(2,2,3);
%     stem(c_h{3});
%
%     subplot(2,2,4);
%     stem(c_h{4});
% end
