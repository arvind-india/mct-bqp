addpath('/home/pedro/rcnn');
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
if strcmp(order, 'toolbox')
  % NOTE: this was in a wrong order, we had to do our own cust bbGt function
  for i=1:length(cameras)
      cameraListImages{i} = bbGt('getFiles',image_directories(i));
  end
elseif strcmp(order, 'cardinal')
  for i=1:length(cameras)
      cameraListImages{i} = loadImages(cameras, image_directories{i}, 0, i, 'campus2');
  end
end

% Use the trained Neural Network to do pedestrian detection (transfer learning)
%sample_size = 300;
sample_size = 12;
allDetections = CNNdetect(cameraListImages, sample_size);

% allDetections are all detections on each frame


%%=========================================================
inplanes{1} = [1 436; 1022 409; 1022 766; 0 766]; % Alameda cam, these points were given to us
inplanes{2} = [3 391; 328 391; 384 295; 475 295; 988 550; 930 622; 1 688]; % Central cam, these points were given to us
% Parse for systematic error detections and remove all empty cells
allDetections = filterCNN(allDetections, inplanes);
for i=1:2
    allDetections{i} = allDetections{i}(~cellfun('isempty',allDetections{i}));
end
% 3D plot over frames
identifyPotentialStaticObjects(allDetections);
%%=======================================================
% Plot images to show detections (with bounding boxes)
figure; show_detections = '4frames';
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
colors = {'Red','Green'};
for i=1:length(cameras)
    drawPoly(ground_plane_regions{i},colors{i},0.5,false);
end


%%==============================================================
num_frames = [10 10]; % Number of frames
start_frames = [1 1+offset_frames]; % Frames to start collecting images

% Get subset of files from the cameraList sequence
for i=1:length(cameras)
    cameraListImages{i} = cameraListImages{i}(:,start_frames(i):(start_frames(i)+num_frames(i)));
    for j=1:length(cameraListImages{i})
      cameraListImages{i}{j} = imread(cameraListImages{i}{j});
    end
end

%%=========================================================
fprintf('Starting tracking loop:\n');
% Global iteration loop
k = g_candidates ^ 2; % Candidates per target
valid_pairing_detections_c1 = {}; valid_pairing_detections_c2 = {};
tau = 3;
groups = {};

for f = 1:(num_frames-1)
    fprintf(['Frame ' num2str(f) ':\n']);
    n = 0; % Number of targets in all cameras
    k_total = 0; % Number of candidates in all cameras
    targs = {}; % Actual targets from both cameras
    cands = {}; % Candidates from both cameras
    
    for c = 1:length(cameras)
        a = allDetections{c}{start_frames(c)+f};
        b = allDetections{c}{start_frames(c)+(f+1)}; % Get the next frame and its detections (i.e our candidates) for association with our targets
        
        targs{c} = zeros(size(a,1),9); 
        cands{c} = zeros(size(b,1),9);
        % Map targets at frame f or f+1 to the ground plane using homographies
        for i=1:size(targs{c},1)
            targ = a(i,:);
            t = H_alt(homographies{c},[targ(3)+targ(5)/2 targ(4)+targ(6)]);
            targs{c}(i,:) = [targ t(1) t(2)];
        end
        for j=1:size(cands{c},1)
            cand = b(j,:);
            t = H_alt(homographies{c},[cand(3)+cand(5)/2 cand(4)+cand(6)]);
            cands{c}(j,:) = [cand t(1) t(2)];
        end
    end

    n1 = size(targs{1},1); n2 = size(targs{2},1); n = n1 + n2;
    k1 = size(cands{1},1); k2 = size(cands{2},1); k_total = k1 + k2; %k is already defined
    % Initialize tracks
    if f == 1
      tracks_c1 = cell(n1,1);tracks_c2 = cell(n2,1);
      id = 1;
      for i =1:n1
        tracks_c1{i} = [id f 1000 targs{1}(i,8:9) 0 0];
        id = id + 1;
      end
      for i = 1:n2
        tracks_c2{i} = [id f 2000 targs{2}(i,8:9) 0 0];
        id = id + 1;
      end
      tracks = vertcat(tracks_c1, tracks_c2);
      first_tracks = tracks;
    end

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
      drawBBs(targs{c}(:,3:7), 'Yellow', 'campus_2'); % 3:7 because 7th position has the score from the CNN
      title(['Targets in camera ' num2str(cameras{c})]);
      % Draw the candidates
      subplot(3,1,2), subimage(cameraListImages{c}{start_frames(c)+(f+1)});
      title(['Candidates in camera ' num2str(cameras{c})]);
      drawCandidateBBs(cands{c}(:,3:6), 'Green', 'campus_2', k);
      [c_a, weights, Z, y] = appearanceConstraint_v2(k,size(cands{c},1),cands{c},cameraListImages{c}{start_frames(c)+(f+1)},'naive',lambda);
      c_as{c} = c_a;
      [~,m] = min(c_a');
      hold on
      for i=1:length(m)
        x_c = fix(m(i)/g_candidates);
        y_c = m(i)-1 - (x_c) * g_candidates;
        cx = cands{c}(i,3) + (x_c-2) * cands{c}(i,5)/delta;
        cy = cands{c}(i,4) + (y_c-2) * cands{c}(i,6)/delta;
        rectangle('Position',[cx cy cands{c}(i,5:6)],'EdgeColor', 'Magenta', 'LineWidth', 1);
      end
    end




    % TODO make this generic for more than 2 cameras
    c_a = [repmat(c_as{1},1,n1) repmat(c_as{2},1,n2)];






    % TODO we ignore the motion and neighbourhood, and neighbourhood motion cues here for now
    Cg = zeros(n*k,n*k);
    % For some reason Cg often has huge negative values so we normalize them for sanity sake
    normCg = Cg - min(Cg(:));
    if max(normCg(:)) ~= 0
      normCg = normCg ./ max(normCg(:));
    else
      normCg = zeros(n*k,n*k);
    end
    
    %---------------------------------------------------------------------------
    fprintf('\t Solving Frank-Wolfe optimization...\n');
    % Prepare inputs for Frank Wolfe (conditional gradient)
    [H_,F,Aeq,Beq,labels] = FW_preamble(n,k,c_a,c_m,c_nm,normCg);
    % Solve the problem using Frank Wolfe
    [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(H_,F,Aeq,Beq,labels); % minx is the value we want
    optimization_results = reshape(minx,k,[]);
    %NOTE: Two candidates may be assigned to the same target
    fprintf('\t Found optimal candidates for each target.\n');
    %---------------------------------------------------------------------------
    


    kill;
end





% Plot pedestrians
plotDetectionsGroundPlane(allDetections,homographies,ground_plane_regions,'show_outliers','campus_2');
% Plot overlap of camera regions
[overlap, ~, ~] = computeOverlap(ground_plane_regions);
drawPoly(overlap,'Black',1.0,false);
