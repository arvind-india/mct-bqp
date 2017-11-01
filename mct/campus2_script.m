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
cameraListImages = cell(2,1);
% NOTE: this was in a wrong order, we had to do our own cust bbGt function
%for i=1:length(cameras)
%    cameraListImages{i} = bbGt('getFiles',image_directories(i));
%end
for i=1:length(cameras)
    cameraListImages{i} = bb_getImages(image_directories{i});
end
% Use the trained Neural Network to do pedestrian detection
%sample_size = 300;
sample_size = 12;
allDetections = CNNdetect(cameraListImages, sample_size);
%%=========================================================
inplanes{1} = [1 436; 1022 409; 1022 766; 0 766]; % Alameda cam, these points were given to us
inplanes{2} = [3 391; 328 391; 384 295; 475 295; 988 550; 930 622; 1 688]; % Central cam, these points were given to us
% Parse for systematic error detections and remove all empty cells
allDetections = filterCNN(allDetections, inplanes);
for i=1:2
    allDetections{i} = allDetections{i}(~cellfun('isempty',allDetections{i}));
end
%%=======================================================
% Plot images to show detections (with bounding boxes)
figure; show_detections = 'slideshow';
if strcmp('slideshow', show_detections) == 1

    start_frame = 1;
    for start_frame = start_frame:4:sample_size
        waitforbuttonpress;
        clf;
        plotDebugBoundingBoxes(cameraListImages,allDetections,start_frame,'campus_2');
    end
elseif strcmp('4frames', show_detections) == 1
    start_frame = 1;
    plotDebugBoundingBoxes(cameraListImages,allDetections,start_frame,'campus_2');
end
%%=======================================================
% Plot detections of all peds in both cameras (we consider that peds are represented by the middle of their BB's)
plotDetectionsCameraSpace(cameraListImages,allDetections,'campus_2');
%%=======================================================
% Load homographies, these homographies where obtained via the use of cp2tform
[homographies, invhomographies] = loadHomographies(homography_directory,'campus_2'); % Defined in global variables
% Plot regions
%inplanes{1} = [494 545; 426 687; 602 681; 852 590; 631 539]; % Alameda cam
%inplanes{2} = [162 510; 702 608; 917 558; 603 390; 447 412]; % Central cam
%inplanes{1} = [1 436; 1022 409; 1022 766; 0 766]; % Alameda cam
%inplanes{2} = [3 391; 328 391; 384 295; 475 295; 988 550; 930 622; 1 688]; % Central cam
ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'campus_2');

%%==============================================================
% POM
figure
hold on;
gplane = [-1000 1000 1000 -1000; 400 400 -1200 -1200];
drawGrid(gplane, 50, 'campus_2');

%%==============================================================

colors = {'Red','Green'};
for i=1:length(cameras)
    drawPoly(ground_plane_regions{i},colors{i},0.5,false);
end
% Plot pedestrians
plotDetectionsGroundPlane(allDetections,homographies,ground_plane_regions,'show_outliers','campus_2');
% Plot overlap of camera regions
[overlap, ~, ~] = computeOverlap(ground_plane_regions);
drawPoly(overlap,'Black',1.0,false);

%%==============================================================
% Single camera preamble for tracking (uses grouping, appearance, cues, etc...) + uses ILOG CPLEX for Frank-Wolfe optimization
predictions = {}; all_predictions = {}; past_observations = {};
cam_id = 1; %Camera id being considered in this iteration
frame_number = 10; %Number of frames
start_frame = 1;
% Total number of sampled candidates for each target
homography_constraint = false;
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
    [c_a, allbbs] = appearanceConstraint(n,k,f,allDetections,cameraListImages,lambda,'naive','campus_2');

    % Spatial Proximity Constraint
    Csp = spatialproximityConstraint(n,k,allbbs);

    % Group Constraint
    Cg = groupConstraint(n,k,f,allbbs,allDetections);

    % Motion Constraint
    c_m = motionConstraint(n,k,f,fps,allDetections,predictions,past_observations);

    % Neighbourhood motion Constraint - all targets are neighbours since we are not in a crowded scenery
    c_nm = neighbourhoodMotionConstraint(n,k,f,fps,allDetections,predictions,past_observations);

    [A,b,Aeq,Beq,labels] = FW_preamble(n,k,c_a,c_m,c_nm,Csp,Cg);

    % Solve the problem
    [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(A,b, Aeq, Beq, labels); % minx is the value we want

    % Get chunk of k candidates for target i and see which one was picked
    optimization_results = reshape(minx,k,[]);

    past_observations = {}; % Erase past observations
    for target = 1:n
        [r,opt_col] = find(optimization_results(:,target));
        best_candidate_pos = allbbs{target}(r,:);
        % Compute velocity
        target_pos = allDetections{1}{f}(target,:);
        % Store this frame's targets with their speed + pos to be used in motion
        velocity = [((target_pos(3) + target_pos(5)/2) - (best_candidate_pos(1) + best_candidate_pos(3)/2))/(1.0/fps) ((target_pos(4) + target_pos(6)/2) - (best_candidate_pos(2) + best_candidate_pos(4)/2))/(1.0/fps)];
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
