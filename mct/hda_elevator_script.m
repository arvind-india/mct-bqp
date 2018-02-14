% Set params of the detector (ACF in this case) and for processing data
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
num_frames = [30 30]; % Number of frames
start_frames = [1336 1200]; % Frames to start collecting images
cameraListImages = loadImages(cameras, image_directory, num_frames, start_frames, 'hda');

%%=========================================================
% Plot 4 sample_size images to show detections (with bounding boxes)
%plotDebugBoundingBoxes(cameraListImages,allDetections,start_frame,'hda');
% %%=======================================================
fprintf('Loading ground truth...\n');

gt = cell(length(cameras),1);
for c=1:length(cameras)
  nmVbb = [hdaRootDirectory '/hda_annotations/cam' int2str(cameras{c}) '.txt'];
  annotations = vbb('vbbLoad', nmVbb);
  gt{c} = annotations.objLists;
end


















fprintf('Plotting ground truth...\n');
number_of_all_peds = 80;
targ_bins_c1 = cell(number_of_all_peds,1); % We have like less than 80 ped ids right?
for i = 1:num_frames(1)
  actual_frames = start_frames(1) + i;
  a = allDetections{1}{actual_frames};
  for tr = 1:size(a,1)
    targ = a(tr,:);
    t = H(homographies{1},[targ(3)+targ(5)/2; targ(4)+targ(6)]);
    tar = [t(1) t(2)];
    targ_bins_c1{a(tr,7)} = vertcat(targ_bins_c1{a(tr,7)},tar);
  end
end
targ_bins_c2 = cell(number_of_all_peds,1); % We have like less than 80 ped ids right?
for i = 1:num_frames(2)
  actual_frames = start_frames(2) + i;
  a = allDetections{2}{actual_frames};
  for tr = 1:size(a,1)
    targ = a(tr,:);
    t = H(homographies{2},[targ(3)+targ(5)/2; targ(4)+targ(6)]);
    tar = [t(1) t(2)];
    targ_bins_c2{a(tr,7)} = vertcat(targ_bins_c2{a(tr,7)},tar);
  end
end
figure
hold on
for i=1:number_of_all_peds
    if isempty(targ_bins_c1{i}) == 0
        scatter(targ_bins_c1{i}(:,1),targ_bins_c1{i}(:,2));
    end
end
figure
hold on
for i=1:number_of_all_peds
  if isempty(targ_bins_c2{i}) == 0
      scatter(targ_bins_c2{i}(:,1),targ_bins_c2{i}(:,2));
  end
end
















%%=========================================================
% Get a ground truth for an experiment, use RunToVisualize to see these
allPedIds = ground_truth;
allPedIds = computeAllPedIds(homographies, allPedIds, cameras);
ped22 = {allPedIds{1}{22}(145:166,:); allPedIds{2}{22}(73:90,:)};
ped24 = {allPedIds{1}{24}(77:79,:); allPedIds{2}{24}(42:44,:)};
%%=========================================================
fprintf('Starting tracking loop:\n');
% Global iteration loop
k = g_candidates ^ 2; % Candidates per target
valid_pairing_detections_c1 = {}; valid_pairing_detections_c2 = {};
tau = 3;
groups = {};
%---------------------------------------------------------------------------
for f = 1:(num_frames-1)
    fprintf(['Frame ' num2str(f) ':\n']);
    if f==2 % DEBUG
      kill2
    end
    %---------------------------------------------------------------------------
    fprintf('\t Getting targets and candidates...\n');
    tic
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
    % Initialize tracks
    if f == 1
      tracks_c1 = cell(n1,1);
      tracks_c2 = cell(n2,1);
      % Create ids and positions and motion models
      % NOTE (id, x, y, vx, vy)
      id = 1;
      for i =1:n1
        tracks_c1{i} = [id f 57 targs{1}(i,9:10) 0 0];
        id = id + 1;
      end
      for i = 1:n2
        tracks_c2{i} = [id f 58 targs{2}(i,9:10) 0 0];
        id = id + 1;
      end
      tracks = vertcat(tracks_c1, tracks_c2);
      first_tracks = tracks;
    else
      if n1 > size(tracks_c1,1)
        disp('cam1 frame has more detections than last frame');
      end
      if n2 > size(tracks_c2,1)
        disp('cam2 frame has more detections than last frame');
      end
    end

    time=toc;
    fprintf(['\t Getting targets and candidates took: ', num2str(round(time*100)/100), ' seconds \n']);
    %---------------------------------------------------------------------------

    fprintf('\t Computing appearance cues...\n');
    % Compute local cues - appearance and motion
    tic
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

      % NOTE: Debug to show histograms
      [~,m] = min(c_a);
      x_c = fix(m/g_candidates);
      y_c = m-1 - (x_c) * g_candidates;
      cx = cands{c}(1,3) + (x_c-2) * cands{c}(1,5)/delta;
      cy = cands{c}(1,4) + (y_c-2) * cands{c}(1,6)/delta;
      hold on
      rectangle('Position',[cx cy cands{c}(1,5:6)],'EdgeColor', 'Magenta', 'LineWidth', 1);

      subplot(3,1,3), bar(1:k,c_a);
      xticks(1:k);

    end
    % TODO make this generic for more than 2 cameras
    c_a = [repmat(c_as{1},1,n1) repmat(c_as{2},1,n2)];

    c_nm = zeros(1,n*k);
    time=toc;
    fprintf(['\t Computing appearance cues took: ', num2str(round(time*100)/100), ' seconds \n']);
    %---------------------------------------------------------------------------
    % Compute global cues - grouping (we ignore spatial proximity cues)
    fprintf('\t Computing motion cues (for frame > 1)\n');
    if f ~= 1 % n*k = (n1+n2)*(k1+k2) = n1*k1 + n1*k2 + n2*k1 + n2*k2
      c_m = [];
      % Get n1 candidates <--> targets motion cues
      sigma = [1 0; 0 1];
      cam1_gaussian_diffs = zeros(n1*k1,1); idx = 1;
      for i=1:n1
        targ = tracks_c1{i};
        center = [targ(9) + targ(11)/dt; targ(10) + targ(12)/dt];
        targ_gaussian = gauss2d(g_candidates, sigma, center);

        %instead of having the 25 sliding boxes per candidate get a gaussian that is fixed on the targets + targets dx/dy
        %and do the absolute difference to a gaussian centered on each candidate
        for j=1:k1
          % Generate a gaussian for this candidate
          center = [cands{1}(9) + cands{1}(11)/dt; cands{1}(10) + cands{1}(12)/dt];
          cand_gaussian = gauss2d(g_candidates, sigma, center);
          % Subtract and absolute the 2 gaussians
          cam1_gaussian_diffs(idx) = pdist2(targ_gaussian, cand_gaussian,'chisq');
          idx = idx + 1;
        end
      end
      cm = horzcat(cm,cam1_gaussian_diffs');
      cm = horzcat(cm,zeros(1,n1*k2));

      % Get n2 candidates <--> targets motion cues
      cam1_gaussian_diffs = zeros(n1*k1,1); idx = 1;
      for i=1:n2
        targ = tracks_c2{i};
        for j=1:k2

        end
      end
      cm = horzcat(cm,zeros(1,n2*k1));
      cm = horzcat(cm,cam2_gaussian_diffs');

    else
      c_m = zeros(1,n*k);
    end


    fprintf('\t Computing neighbourhood motion cues (for frame > 1)\n');


    %---------------------------------------------------------------------------
    % Compute global cues - grouping (we ignore spatial proximity cues)
    fprintf('\t Computing grouping cues...\n');
    tic
    fprintf('\t Creating/Updating groups\n');
    %if f==1 || mod(f,tau) == 0
    %  % Add targets to group
    %  for i_t = 1:n
    %    % Get target
    %    t_x = targs(i_t,9);
    %    t_y = targs(i_t,10);
    %    in_group = 0:
    %    for grp = 1:length(groups)
    %      for ele = 1:length(groups{grp})
    %        % Is this element from this group within the group_bubble?
    %        x = groups{grp}(ele,9);
    %        y = groups{grp}(ele,10);
    %        if sqrt(tx-x)^2 + (ty-y)^2) < 1.2
    %          in_group = 1;
    %          groups{grp} = vertcat(groups{grp}, targs(i_t,:));
    %          % TODO early break
    %      end
    %    end
    %    if in_group == 0
    %      % Create a single group for this target
    %      group{end+1} = targs(i_t,:);
    %    end
    %  end
    %end
    time=toc;
    tic
    % NOTE Encourage the selection of candidate locations that keep the formation of targets within each group
    Cg = groupConstraint_v2(n,k,targs);
    %Cg = groupConstraint_v3(n,k,groups,targs,cands);

    %Cg = zeros(n*k,n*k);
    % For some reason Cg often has huge negative values so we normalize them for sanity sake
    normCg = Cg - min(Cg(:));
    if max(normCg(:)) ~= 0
      normCg = normCg ./ max(normCg(:));
    else
      normCg = zeros(n*k,n*k);
    end
    time=toc;

    fprintf(['\t Computing grouping cues took: ', num2str(round(time*100)/100), ' seconds \n']);

    %---------------------------------------------------------------------------
    fprintf('\t Solving Frank-Wolfe optimization...\n');
    % Prepare inputs for Frank Wolfe (conditional gradient)
    [H_,F,Aeq,Beq,labels] = FW_preamble(n,k,c_a,c_m,c_nm,normCg);
    % Solve the problem using Frank Wolfe
    [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(H_,F,Aeq,Beq,labels); % minx is the value we want
    % Get chunk of k candidates for target i and see which one was picked

    optimization_results = reshape(minx,k,[]);
    %NOTE: Two targets may be assigned to the same target
    fprintf('\t Found optimal candidates for each target.\n');
    %-------------------------------------------------------------------------------
    fprintf('\t Creating new tracks (inter-frame assignment)\n');
    figure;
    bar(minx, 'r');
    new_tracks = {};
    % Get first n1 chunks of minx
    n1_chunks = minx(1:n1*k); j = 1;
    for i=1:k:n1*k
      for cn =1:k2
        chunk = n1_chunks(i:(i+k-1));
        [~,m] = max(chunk); % NOTE Find the 1 fast
        x_c = fix(m/g_candidates);
        y_c = m-1 - (x_c) * g_candidates;
        cx = cands{1}(cn,3) + (x_c - 2) * cands{1}(cn,5)/delta;
        cy = cands{1}(cn,4) + (y_c - 2) * cands{1}(cn,6)/delta;
        % Transform the candidate to the groundplane
        t = H(homographies{1},[cx + cands{1}(cn,5)/2; cy + cands{1}(cn,6)]);



        new_tracks{end+1} = [j f+1 cameras{1} t(1) t(2) 0 0];
        j = j+1;
      end
    end
    % Get the remaining n2 chunks of minx
    n2_chunks = minx(n1*k:end); j = 1;
    for i=1:k:n2*k
      for cn =1:k2
        chunk = n2_chunks(i:(i+k-1));
        [~,m] = max(chunk); % NOTE FInd the 1 fast
        x_c = fix(m/g_candidates);
        y_c = m-1 - (x_c) * g_candidates;
        cx = cands{2}(cn,3) + (x_c-2) * cands{2}(cn,5)/delta;
        cy = cands{2}(cn,4) + (y_c-2) * cands{2}(cn,6)/delta;
        % Transform the candidate to the groundplane
        t = H(homographies{2},[cx + cands{2}(cn,5)/2; cy + cands{2}(cn,6)]);

        % Get previous tracked valued so we can compute Velocity

        new_tracks{end+1} = [j f+1 cameras{2} t(1) t(2) 0 0];
        j = j+1;
      end
    end
    new_tracks = new_tracks';
    %-------------------------------------------------------------------------------
    fprintf('\t Solving inter-camera assignment...\n');
    %% Pedestrian/Camera association (we do assymetric target association)
    assignmentAlgorithm = 'jonker_volgenant';
    S = zeros(n1,n2); images = {cameraListImages{1}{start_frames(1)+f},cameraListImages{2}{start_frames(2)+f}};
    [assignments, S, A, P, P_nonnormalized, V] = solve_assignment_v2(S, images, targs, assignmentAlgorithm);
    %[assignments, S] = solve_assignment_v2(S, images, targs, assignmentAlgorithm);
    % NOTE: Debug
    wrong_assignments = 0;
    % TODO many of these loops could probably be turned into parfors...
    for i=1:size(S,1)
      fprintf('\t\tDEBUG: Best assignment for ped %d in cam %d === ped %d in cam %d \n', targs{1}(i,8), targs{1}(i,1),...
      targs{2}(assignments(i),8), targs{2}(assignments(i),1));
      fprintf('\t\tBest assignment for id %d in cam %d === id %d in cam %d \n', i, targs{1}(i,1),...
      assignments(i), targs{2}(assignments(i),1));
      score = S(i,assignments(i));
      fprintf('\t\tScore: %f\n', score);
      % NOTE If this is considered valid, otherwise it could be a flawed assignment
      if score < score_threshold
        if tracks_c1{i}(1) > tracks_c2{assignments(i)}(1)
          tracks_c1{i}(1) = tracks_c2{assignments(i)}(1);
        else
          tracks_c2{assignments(i)}(1) = tracks_c1{i}(1);
        end
        valid_pairing_detections_c1{end+1} = targs{1}(i,:);
        valid_pairing_detections_c2{end+1} = targs{2}(assignments(i),:);
        fprintf('\t\tFixing groups \n');
        % Fixing groups
        for gi=1:length(groups)
          grp = groups{gi};
          for ii = 1:size(grp,1)
            for jj = 1:size(grp,1)
              % If these 2 are from different cameras
              if grp(ii,3) == 57 && grp(jj,3) == 58
                if grp(ii,1) == i && grp(jj,1) == assignment(i)
                  % Eliminate both, store the average position
                  xx1 = grp(ii,9);
                  yy1 = grp(ii,10);
                  xx2 = grp(jj,9);
                  yy2 = grp(jj,10);
                  % Eliminate one
                  grp(ii,:) = [];
                  % replace the others x and y positions
                  grp(jj,9) = (xx1 + xx2)/2.0;
                  grp(jj,10) = (yy1 + yy2)/2.0;
                end
              end
            end
          end
        end
      end
      if targs{1}(i,8) ~= targs{2}(assignments(i),8)
        wrong_assignments = wrong_assignments + 1;
      end
    end
    fprintf('\t\tWrong assignments: %d\n', wrong_assignments);
    fprintf('\tFixing tracks + detections in current frame\n');
    % TODO Remaking tracks
    tracks = vertcat(tracks_c1, tracks_c2);





    %-------------------------------------------------------------------------------
    fprintf('\tCorrecting homographies\n');
    % NOTE Use the valid pairings found previously

    cam1_camdetections = cell2mat(valid_pairing_detections_c1');
    cam1_camdetections = horzcat(cam1_camdetections(:,3) + cam1_camdetections(:,5)/2,cam1_camdetections(:,4) + cam1_camdetections(:,6));
    cam2_camdetections = cell2mat(valid_pairing_detections_c2');
    cam2_camdetections = horzcat(cam2_camdetections(:,3) + cam2_camdetections(:,5)/2,cam2_camdetections(:,4) + cam2_camdetections(:,6));
    %cam1_region = outlier_removal_regions{1};
    %cam2_region = outlier_removal_regions{2};
    % TODO fix this in the initial part
    cam1_region = [4 796; 1022 798; 353 473; 142 431];
    cam2_region = [59 796; 503 353; 1015 375; 1011 798];

    [H1,H2] = correct_homographies(homographies{1}, homographies{2},cam1_camdetections', cam1_region, cam2_camdetections', cam2_region);
    homographies{1} = H1;
    homographies{2} = H2;

    %---------------------------------------------------------------------------

    tracks = cell2mat(tracks);
    tracks_c1 = cell2mat(tracks_c1);
    tracks_c2 = cell2mat(tracks_c2);
    new_tracks = cell2mat(new_tracks);
    new_tracks = accumarray(new_tracks(:,3),(1:size(new_tracks,1)).',[],@(x){new_tracks(x,:)},{});

    disp('Calculating new velocities...');
    % Calculating new velocities
    for n=1:size(new_tracks{57},1)
      for m = 1:size(tracks_c1,1)
        if tracks_c1(m,2) == f
          % Get previous positions
          if tracks_c1(m,1) == new_tracks{57}(n,1)
            new_tracks{57}(n,6) = tracks_c1(m,4) - new_tracks{57}(n,4);
            new_tracks{57}(n,7) = tracks_c1(m,5) - new_tracks{57}(n,5);
          end
        end
      end
    end
    for n=1:size(new_tracks{58},1)
      for m = 1:size(tracks_c2,1)
        if tracks_c2(m,2) == f
          % Get previous positions
          if tracks_c2(m,1) == new_tracks{58}(n,1)
            new_tracks{58}(n,6) = tracks_c2(m,4) - new_tracks{58}(n,4);
            new_tracks{58}(n,7) = tracks_c2(m,5) - new_tracks{58}(n,5);
          end
        end
      end
    end

    %---------------------------------------------------------------------------
    % NOTE Debug
    disp('Showing tracks. Previous tracks -> points with black border + black line. \n New tracks -> points with gray border + gray line');
    figure
    % NOTE We expect at most around 10 pedestrians so 10 colours should be more than enough
    colours = {'Yellow', 'Green', 'Pink', 'Purple', 'Grey', 'Orange'};
    hold on
    tracks_c1 = vertcat(tracks_c1,new_tracks{57});
    tracks_c2 = vertcat(tracks_c2,new_tracks{58});
    for t=1:size(tracks,1)
      scatter(tracks(t,4),tracks(t,5),'MarkerFaceColor',rgb(colours{tracks(t,1)}),'MarkerEdgeColor',rgb(colours{tracks(t,1)}));
    end
    figure
    hold on
    % Incorporate new tracks
    tracks_c1 = accumarray(tracks_c1(:,1),(1:size(tracks_c1,1)).',[],@(x){tracks_c1(x,:)},{});
    tracks_c2 = accumarray(tracks_c2(:,1),(1:size(tracks_c2,1)).',[],@(x){tracks_c2(x,:)},{});
    for t=1:size(tracks_c1,1)
      plot(tracks_c1{t}(:,4),tracks_c1{t}(:,5),'k');
      for d=1:size(tracks_c1{t},1)
        if tracks_c1{t}(d,2) == 1
          scatter(tracks_c1{t}(d,4),tracks_c1{t}(d,5),'MarkerFaceColor',rgb(colours{t}),'MarkerEdgeColor',rgb('Blue'));
        else
          scatter(tracks_c1{t}(d,4),tracks_c1{t}(d,5),10,'MarkerFaceColor',rgb(colours{t}),'MarkerEdgeColor',rgb('Blue'));
        end
      end
    end
    for t=1:size(tracks_c2,1)
      plot(tracks_c2{t}(:,4),tracks_c2{t}(:,5),'k');
      for d=1:size(tracks_c2{t},1)
        if tracks_c2{t}(d,2) == 1
          scatter(tracks_c2{t}(d,4),tracks_c2{t}(d,5),'MarkerFaceColor',rgb(colours{t}),'MarkerEdgeColor',rgb('Blue'));
        else
          scatter(tracks_c2{t}(d,4),tracks_c2{t}(d,5),10,'MarkerFaceColor',rgb(colours{t}),'MarkerEdgeColor',rgb('Blue'));
        end
      end

    end


    %kill
    %---------------------------------------------------------------------------
end
