addpath('/home/pedro/rcnn');
set(0,'DefaultFigureVisible','off');
setDetectionParams_hda_hall;
setTrackerParams;

gnd_detections = load_data('hda', cameras);
% Load the images (needed for the appearance cues)
cameraListImages = cell(2,1); inplanes = cell(2,1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, durations(i), 0, 'hda');
    inplanes{i} = load(strcat(visibility_regions_directory,num2str(cameras{i}),'.mat')); inplanes{i} = inplanes{i}.t;
end
[homographies, invhomographies] = loadHomographies(homography_directory,'hda', cameras);
ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'hda');
[overlap, ~, ~] = computeOverlap(ground_plane_regions);
ground_plane_regions_adjusted = cell(2,1);
%%=========================================================
fprintf('Starting tracking loop:\n');
valid_matchings = cell(2,1);
groups = {}; adjusted_positions = cell(2,1); tracklets = {};
num_frames = 7; % Number of frames
start = 6811;
start_frames = [start + offset_frames start]; % Frames to start collecting images
k = g_candidates ^ 2; % Candidates per target
tau = 1; psi = 8; xi = 1;
N = 0; % Number of targets in all cameras
penalize_val = 1;
reward_val = -1;
score_threshold = 0.4;
comfort_distance = 1.0; % Anyone closer than this meters is in talking range?
initial_speed_x = 50;
initial_speed_y = 0;
a_sigma = sqrt(0.6); m_sigma = [15 0; 0 15]; G_sigma =  2 * (2 ^ 2);
for f = 1:(num_frames - 1)
    if f == 5
        break;
    end
    %---------------------------------------------------------------------------
    fprintf(['Frame ' num2str(f) ':\n']);
    %---------------------------------------------------------------------------
    fprintf('\t Getting images...\n');
    next_images = cell(length(cameras),1); images = cell(length(cameras),1);
    for i = 1:length(cameras)
        next_images{i} = imread(cameraListImages{i}{start_frames(i)+(f+1)});
        images{i} = imread(cameraListImages{i}{start_frames(i)+(f)});
    end
    %---------------------------------------------------------------------------
    fprintf('\t Getting targets...\n');
    if f == 1
        targs = cell(2,1); % TODO Actual targets from both cameras
        for id = 1:length(cameras)
            targs{id} = gnd_detections{id}{start_frames(id) + f};
        end
        targs = cell2mat(targs);
        % targs = cell2mat(targs'); % For some reason this work on one computer and on the other one does not
        % TODO If the targets are empty on the first frame, can't track
        if isempty(targs)
            error('Cannot possibly track if there are no detections on the first given frame');
        end
    end

    targs_percam = (accumarray(targs(:,1),(1:size(targs,1)).',[],@(x){targs(x,:)},{}));
    N = size(targs,1);
    n1 = size(targs_percam{1},1);
    n2 = size(targs_percam{2},1);
    cands = cell(N,1); cands_homo = cands; % Candidates from both cameras for each target, always empty each frame
    cands_percam = cell(2,1); cands_homo_percam = cands_percam;
    %---------------------------------------------------------------------------
    % TODO Sample around the targets. We then use these candidates on the next frame
    fprintf('\t Sampling candidates...\n');
    for t = 1:size(targs,1)
        t_pos = targs(t,8:9);
        t_rect = targs(t,4:7);
        cands{t} = zeros(k,4);
        counter = 1;
        for gridx=-floor(g_candidates/2):floor(g_candidates/2)
          for gridy=-floor(g_candidates/2):floor(g_candidates/2)
            sampling_dx = t_rect(3)/20;
            sampling_dy = t_rect(4)/20;
            % TODO Normal candidates
            cx = t_rect(1) + gridx * sampling_dx;% startx + gridx * xstep
            cy = t_rect(2) + gridy * sampling_dy; % starty + gridy * ystep
            cands{t}(counter,:) = [cx cy t_rect(3) t_rect(4)];
            % TODO Compute homography transformation of candidates and store them in cand_homo
            cands_homo{t}(counter,:) = transpose(H(homographies{targs(t,1)}, [cx+t_rect(3)/2; cy+t_rect(4)]));
            counter = counter + 1;
          end
        end
        cands_percam{targs(t,1)}{end+1} = cands{t}; cands_homo_percam{targs(t,1)}{end+1} = cands_homo{t};
    end
    %---------------------------------------------------------------------------
    % TODO check for potential new targets to add from the detections
    fprintf('\t Checking for potential new targets...\n');

    % Check if detections are available. If there are more detections than currently there are targets consider this
    if f ~= 1
        if size(gnd_detections{id}{start_frames(id) + f},1) > size(targs_percam{id},1)
            fprintf('\t \t More detections than targets! \n');
        end
    end

    %---------------------------------------------------------------------------
    % TODO store the ones that are ambiguous for homography correction in targs_in_overlap (i.e gating part 1)
    fprintf('\t Checking for targets in the overlapping regions...\n');

    targs_in_overlap = {};
    motion_models_overlap = {};
    for t = 1:size(targs,1)
        if polyin([targs(t,8) targs(t,9)],overlap)
            targs_in_overlap{end+1} = targs(t,:);
            if f ~= 1
                motion_models_overlap{end+1} = [targs(t,1) transpose(motion_models{t})];
            end
        end
    end
    if ~isempty(targs_in_overlap)
        if f ~= 1
            motion_models_overlap = cell2mat(motion_models_overlap');
            motion_models_overlap = (accumarray(motion_models_overlap(:,1),(1:size(motion_models_overlap,1)).',[],@(x){motion_models_overlap(x,:)},{}));
        end
        targs_in_overlap = cell2mat(targs_in_overlap');
        targs_in_overlap = (accumarray(targs_in_overlap(:,1),(1:size(targs_in_overlap,1)).',[],@(x){targs_in_overlap(x,:)},{}));
    end
    %---------------------------------------------------------------------------
    % TODO appearance
    fprintf('\t Computing apperance cues...\n');
    a = cell(length(cameras),1);
    for i = 1:length(cameras)
        n = size(targs_percam{i},1);
        [c_a, w, Z, y] = appearance(k,n,targs_percam{i},cands_percam{i},images{i},next_images{i},'naive',lambda,a_sigma);
        a{i} = c_a;
        plotAppeance(c_a,i,n,k,cameraListImages,f,targs_percam,cameras,cands_percam, start_frames,cands_homo_percam);
    end
    a = cell2mat(a);
    a = a./max(abs(a(:))); % TODO This normalization should help?
    %---------------------------------------------------------------------------
    % TODO create motion models
    if f == 1
        fprintf('\t Creating motion models...\n');
        motion_models = cell(N,1);
        for t = 1:size(targs,1)
            motion_models{t} = [targs(t,8); targs(t,9); initial_speed_x; initial_speed_y];
        end
    end
    %--------------------------------
    % TODO motion
    fprintf('\t Computing motion cues...\n');
    m = cell(length(cameras),1);
    for i = 1:length(cameras)
        n = size(targs_percam{i},1);
        c_m = motion(n,k,motion_models,cands_homo_percam{i},fps,m_sigma);
        m{i} = c_m;
        plotMotion(i, c_m, k, n, floor_image, cands_homo_percam);
    end
    m = cell2mat(m);
    m = m./max(abs(m(:))); % TODO This normalization should help?
    %--------------------------------
    % TODO create groups
    fprintf('\t Creating groups...\n');
    if rem(f,tau) == 0
        groups = cell(2,1);
        for i = 1:length(cameras)
                Y = pdist(targs_percam{i}(:,8:9));
                Z = linkage(Y);
                C = cluster(Z,'cutoff',comfort_distance,'criterion','distance');
                groups{i} = C;
        end
    end
    %--------------------------------
    % TODO grouping
    fprintf('\t Computing grouping cues...\n');
    T = grouping(N,k,groups,targs,targs_percam,cands_homo,G_sigma);
    T = T./max(abs(T(:))); % TODO This normalization should help?
    Dinvsq = diag(sum(T,2)).^(-1/2); %row sum
    Dinvsq(~isfinite(Dinvsq)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
    G = eye(N*k) - Dinvsq*T*Dinvsq; %Normalized Laplacian matrix so G is convex
    plotGrouping(T,G);

    %--------------------------------
    % TODO out-of-bounds cue
    b = cell(length(cameras),1);
    for i = 1:length(cameras)
        n = size(targs_percam{i},1);
        c_b = bounds(i,k,n,cands_homo_percam,ground_plane_regions,penalize_val,reward_val);
        plotBounds(i, c_m, k, n, floor_image, cands_homo_percam);
        b{i} = c_b;
    end
    b = cell2mat(b);
    %--------------------------------

    %---------------------------------------------------------------------------
    % TODO join all cues and solve FW
    fprintf('\t Solving Frank-Wolfe optimization...\n');
    % Prepare inputs for Frank Wolfe (conditional gradient)
    [H_,F,Aeq,Beq,labels] = FW_preamble(N,k,a,m,G,b);

    % Solve the problem using Frank Wolfe
    [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(H_,F,Aeq,Beq,labels); % minx is the value we want

    optimization_results = reshape(minx,k,[]); optimization_results_percam = cell(2,1);
    % TODO fix this, should work for multiple cameras
    optimization_results_percam{1} = optimization_results(:,1:n1);
    optimization_results_percam{2} = optimization_results(:,n1:end);

    fprintf('\t Found optimal candidates for each target. If debug option, will now show selected candidates.\n');

    % NOTE Show next frame, show previous target location, show all sampled candidates and show best candidate
    for test_cam = 1:length(cameras)
        figure; hold on;
        title(['t+1 Candidates (BB) in camera ' cameras{test_cam}]);
        imshow(cameraListImages{test_cam}{start_frames(test_cam)+f});
        %drawCandidateBBs(cell2mat(transpose(cands_percam{test_cam})), 'Green', 'hda', k);
        drawBBs(targs_percam{test_cam}(:,4:7), 'Yellow', 'hda');
        for i=1:size(targs_percam{test_cam})
            for j=1:k
                if optimization_results_percam{test_cam}(j,i) == 1
                    % DEBUG
                    rectangle('Position',cands_percam{test_cam}{i}(j,:),'EdgeColor', 'Red', 'LineWidth', 1);
                end
            end
        end
    end
    %#############################################################################################################################
    fprintf('\t Performing inter-camera disambiguation in overlap region...\n');
    if rem(f-1,xi) == 0 && ~isempty(targs_in_overlap) % NOTE Only do this if there are targets in overlap of course
        fprintf('\t Using method 1 -- Matching...\n');
        % NOTE METHOD ONE OF INTER-CAMERA ASSIGNMENT
        % TODO Build Score matrix (how to generalize this to n cameras? NP-hard assignment problem?)
        n_ov1 = size(targs_in_overlap{1},1);
        n_ov2 = size(targs_in_overlap{2},1);
        [S, A, P, V] = createScoreMatrix(f,n_ov1,n_ov2,targs_in_overlap,images,d1_metric,d256_metric, motion_models_overlap);

        % TODO Gating (i.e gating part2)
        for a1 = 1:n_ov1
            for b1 = 1:n_ov2
                if P(a1,b1) > gating_distance
                    S(a1,b1) = Inf;
                end
            end
        end
        % TODO Target Coupling (disambiguate between targets that are in the overlapping region)
        assignments = lapjv(S,eps); % NOTE eps can be changed to accelerate the algorithm
        for i=1:size(S,2)
            if n_ov2 >= n_ov1
                fprintf('\t\tBest assignment for id %d in cam %d === id %d in cam %d \n', i, targs_in_overlap{1}(i,1),...
                assignments(i), targs_in_overlap{2}(assignments(i),1));
                score = S(i,assignments(i));
            else
                fprintf('\t\tBest assignment for id %d in cam %d === id %d in cam %d \n', assignments(i), targs_in_overlap{1}(assignments(i),1),...
                i, targs_in_overlap{2}(i,1));
                score = S(assignments(i),i);
            end

            fprintf('\t\tScore: %f\n', score);
            % TODO If its a good score, then store merge
            if score < score_threshold
                % TODO Make this work for more than 2 cameras
                if n_ov2 >= n_ov1
                    valid_matchings{1}{end+1} = targs_in_overlap{1}(i,:);
                    valid_matchings{2}{end+1} = targs_in_overlap{2}(assignments(i),:);
                else
                    valid_matchings{1}{end+1} = targs_in_overlap{1}(assignments(i),:);
                    valid_matchings{2}{end+1} = targs_in_overlap{2}(i,:);
                end
            end
        end

        %---------------------------------------------------------------------------
        % NOTE METHOD TWO OF INTER-CAMERA ASSIGNMENT
        fprintf('\t Using method 2 -- Optimization...\n');
        if n_ov1 == n_ov2
            % TODO We have the same number of targets in both cameras, no need for ghost targets

            % TODO Use the appearance models of the targets from both cameras

            % TODO Use a distance model of the targets from both cameras

        else
            % TODO We need ghost targets
        end

        %---------------------------------------------------------------------------
        % NOTE homography Correction can be done separately, independently of how the target coupling is solved
        % TODO correct homographies and ALL detections using these homographies
        set(0,'DefaultFigureVisible','on');
        if rem(f-1,psi) == 0 && ~isempty(valid_matchings{1}) && ~isempty(valid_matchings{2})
            v_matchings = cell(2,1);
            for i=1:length(cameras)
                if ~isempty(valid_matchings{i})
                    v_matchings{i} = cell2mat(transpose(valid_matchings{i}));
                end
            end
            fprintf('\t\t Correcting homographies...\n');
            homog_solver = 'svd'; % Method to compute homographies
            [rho_r, rho_d, best_N] = determineRho(v_matchings, inplanes, ground_plane_regions, homog_solver); % Determines good rho values for convergence
            [H1, H2, cam1_dets_gnd, cam2_dets_gnd, cam1_region_gnd, cam2_region_gnd] = homography_correction(v_matchings, inplanes, ...
            ground_plane_regions, homog_solver, best_N, rho_r, rho_d);
            homographies{1} = H1; homographies{2} = H2;

            ground_plane_regions_adjusted{1} = cam1_region_gnd;
            ground_plane_regions_adjusted{2} = cam2_region_gnd;
            [overlap_adjusted, ~, ~] = computeOverlap(ground_plane_regions_adjusted);
            adjusted_positions{1}{end+1} = cam1_dets_gnd;
            adjusted_positions{2}{end+1} = cam2_dets_gnd;
        end
        set(0,'DefaultFigureVisible','off');
    end

    %#############################################################################################################################
    %---------------------------------------------------------------------------
    % TODO update motion models using old targets and predicted targets
    fprintf('\t Updating motion models...\n');
    for i=1:N
        for j=1:k
            if optimization_results(j,i) == 1
                % Update Position
                motion_models{i}(1:2) = cands_homo{i}(j,:);
                % Update velocity
                dt = 1.0/fps;
                motion_models{i}(3:4) = (cands_homo{i}(j,:) - targs(i,8:9))/dt;
            end
        end
    end
    %---------------------------------------------------------------------------
    fprintf('\t Saving results, using results as next targets...\n');
    % TODO Store target <-> candidate pairs so we can plot them later
    % TODO "clear" targets, use the predicted ones as new targets
    for i=1:N
        for j=1:k
            if optimization_results(j,i) == 1
                tracklets{end+1} = [targs(i,:); targs(i,1:3) cands{i}(j,:) cands_homo{i}(j,:)];
                targs(i,4:7) = cands{i}(j,:);
                targs(i,8:9) = cands_homo{i}(j,:);
            end
        end
    end
end
set(0,'DefaultFigureVisible','on');
% TODO Plot final tracks for debug mostly
openfig(floor_image); hold on;

for s = 1:length(tracklets)
    if tracklets{s}(1,1) == 1
        plot(tracklets{s}(:,8),tracklets{s}(:,9),'r-s');
    else
        plot(tracklets{s}(:,8),tracklets{s}(:,9),'b-s');
    end
end
for i = 1:length(cameras)
    adjusted_positions{i} = cell2mat(adjusted_positions{i});
    if i == 1
        scatter(adjusted_positions{i}(:,1),adjusted_positions{i}(:,2),'MarkerFaceColor',rgb('Pink'),'MarkerEdgeColor',rgb('Pink'));
    else
        scatter(adjusted_positions{i}(:,1),adjusted_positions{i}(:,2),'MarkerFaceColor',rgb('Green'),'MarkerEdgeColor',rgb('Green'));
    end
end
% TODO Plot ground truths
colors = {'Orange','Purple','Grey'};
colors_adjusted = {'Red','Blue','Black'};
for i=1:length(cameras)
    drawPoly(ground_plane_regions{i},colors{i},0.5,false); % Draw regions
    drawPoly(ground_plane_regions_adjusted{i},colors_adjusted{i},0.5,false);
end
drawPoly(overlap,colors{3},1.0,false);
drawPoly(overlap_adjusted, colors_adjusted{3},1.0,false);
for f = 1:(num_frames - 1)
    if f == 5
        break;
    end
    truth1 = gnd_detections{1}{start_frames(1) + f};
    truth2 = gnd_detections{2}{start_frames(2) + f};
    for i = 1:size(truth1,1)
        scatter(truth1(:,8),truth1(:,9),'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange'));
    end
    for i = 1:size(truth2,1)
        scatter(truth2(:,8),truth2(:,9),'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple'));
    end
end
