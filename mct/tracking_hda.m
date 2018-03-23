set(0,'DefaultFigureVisible','on');
setDetectionParams_hda_hall; setTrackerParams;

gnd_detections = load_data('hda', cameras); % TODO Load the images (needed for the appearance cues)
cameraListImages = cell(2,1); inplanes = cell(2,1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, durations(i), 0, 'hda');
    inplanes{i} = load(strcat(visibility_regions_directory,num2str(cameras{i}),'.mat')); inplanes{i} = inplanes{i}.t;
end
[homographies, invhomographies] = loadHomographies(homography_directory,'hda', cameras);
ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'hda');
[overlap, ~, ~] = computeOverlap(ground_plane_regions);
%%=========================================================
FW_max_iterations = 5000;  % max number of iteration, I found out the opts.TOL (tolerance?) is more important in the convergence in my problem
FW_duality_gap = 2; % This sets the duality gap and you can change it if you want to change the tolerance for FW convergence
FW_eps = 1e-12; % (eps in matlab = 1e-16) not sure why this is needed
num_frames = 7; % Number of frames
start = 6811;
start_frames = [start + offset_frames start]; % Frames to start collecting images
g_candidates = 9; % basis for the number of candidates
k = g_candidates ^ 2; % Candidates per target
tau = 1; psi = 8; xi = 8;
N = 0; % Number of targets in all cameras
penalize_val = 1;
reward_val = -1;
lambda = 0.5; % variable for the appearance cues
gating_distance = 6.0; % This does actually correspond to 6 meters
score_threshold = 0.4;
comfort_distance = 1.5; % Anyone closer than this meters is in talking range?
initial_speed_x = 10.0;
initial_speed_y = 0;
dx = 9; dy = 50;
Alpha = [0.0 0.0]; % weight of the appearance constraint
Zeta = [1.0 1.0]; % weight of the motion constraint
update_homo = true;
a_sigma = [2 ^ 2 0.0; 0.0 2 ^ 2]; m_sigma = [0.5 0; 0 0.5]; G_sigma =  2 * (2 ^ 2);
rho_d = 10;
rho_r = 1;
N_h = 100; % Number of iterations
homog_solver = 'svd'; % Method to compute homographies, NOTE must be either 'svd' or 'ransac'
weights = cell(2,1);
%%=========================================================
debug_test_frames = 2; % DEBUG test these frames
show_ground_truth = true;
show_candidates = false;
show_predicted_bbs = false;
draw_regions = false;
sampling_plane = 'camera'; % NOTE Must be either 'camera' or 'ground'
homocorrec_debug = 'no_debug';
candidates_frame = 2;
debug_gnd_truth_frames = 1;
%%=========================================================
% Frames at which detections are available/allowed
detection_frames = [start_frames; start_frames + 9; start_frames + 15]; % These are used to updated targets/number of targets at these frames
%%=========================================================
fprintf('Starting tracking loop:\n');
for update_homo = 0:1 % DEBUG merely for debug, would never use this is "production"
    fprintf(['Updating homography: ' num2str(update_homo) '\n']);
    all_candidates = cell(num_frames,1); % Store all candidates (in gnd plane)
    ground_plane_regions_adjusted = cell(2,1);
    valid_matchings = cell(2,1);
    groups = {}; adjusted_positions = cell(2,1); tracklets = {};

    for f = 1:(num_frames - 1)
        if f == debug_test_frames+1
            break;
        end
        fprintf(['  Frame ' num2str(f) ':\n \t Getting images...\n']);
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
            targs = cell2mat(targs); % For some reason this work on one computer and on the other one does not
            % TODO If the targets are empty on the first frame, can't track
            if isempty(targs)
                error('Cannot possibly track if there are no detections on the first given frame');
            end
        end

        targs_percam = (accumarray(targs(:,1),(1:size(targs,1)).',[],@(x){targs(x,:)},{}));
        N = size(targs,1);
        n1 = size(targs_percam{1},1); n2 = size(targs_percam{2},1);
        cands = cell(N,1); cands_percam = cell(2,1); % Candidates from both cameras for each target, always empty each frame
        %---------------------------------------------------------------------------
        % TODO Sample around the targets. We then use these candidates on the next frame
        fprintf('\t Sampling candidates...\n');
        for t = 1:size(targs,1)
            t_pos = targs(t,8:9);
            t_rect = targs(t,4:7);
            cands{t} = zeros(k,6);
            counter = 1;
            for gridx = -floor(g_candidates/2):floor(g_candidates/2)
              for gridy = -floor(g_candidates/2):floor(g_candidates/2)
                if strcmp(sampling_plane,'camera')
                    sampling_dx = t_rect(3)/dx;
                    sampling_dy = t_rect(4)/dy;
                    cx = t_rect(1) + gridx * sampling_dx;
                    cy = t_rect(2) + gridy * sampling_dy;
                    cands{t}(counter,:) = [cx cy t_rect(3:4) transpose(H(homographies{targs(t,1)}, [cx+t_rect(3)/2; cy+t_rect(4)]))];
                elseif strcmp(sampling_plane,'ground')
                    sampling_dx = 0.05; % TODO Bernardino proposed a way to actually get bb width from gnd plane
                    sampling_dy = 0.02; % TODO but we would need some data we do not have :(
                    g_cx = t_pos(1) + gridx * sampling_dx;
                    g_cy = t_pos(2) + gridy * sampling_dy;
                    c_pos = transpose(H(invhomographies{targs(t,1)}, [g_cx; g_cy]));
                    cands{t}(counter,:) = [c_pos(1) c_pos(2) t_rect(3:4) g_cx g_cy];
                end
                counter = counter + 1;
              end
            end
            cands_percam{targs(t,1)}{end+1} = cands{t};
        end
        all_candidates{f} = cands_percam;
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
        fprintf('\t Checking for targets in the overlapping regions...\n')
        targs_in_overlap = {}; motion_models_overlap = {};
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
            [c_a, w, Z_app, y] = appearance(k,n,targs_percam{i},cands_percam{i},images{i},...
                next_images{i},'naive',lambda,a_sigma,dx,dy, g_candidates, weights{i});
            a{i} = c_a;
            % TODO Store weights which might be useful
            weights{i} = w;
            %plotAppeance(c_a,i,n,k,cameraListImages,f,targs_percam,cameras,cands_percam, start_frames,cands_percam);
        end
        a = cell2mat(a);
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
            c_m = motion(n,k,motion_models,cands_percam{i},fps,m_sigma);
            m{i} = c_m;
            %plotMotion(i, c_m, k, n, floor_image, cands_percam);
        end
        m = cell2mat(m);
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
        T = grouping(N,k,groups,targs,targs_percam,cands,G_sigma);
        Dinvsq = diag(sum(T,2)).^(-1/2); %row sum
        Dinvsq(~isfinite(Dinvsq)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
        G = eye(N*k) - Dinvsq*T*Dinvsq; %Normalized Laplacian matrix so G is convex
        %plotGrouping(T,G);
        %G = zeros(N*k);
        %--------------------------------
        % TODO out-of-bounds cue
        b = cell(length(cameras),1);
        for i = 1:length(cameras)
            n = size(targs_percam{i},1);
            c_b = bounds(i,k,n,cands_percam,ground_plane_regions,penalize_val,reward_val);
            %plotBounds(i, c_m, k, n, floor_image, cands);
            b{i} = c_b;
        end
        b = cell2mat(b);
        %---------------------------------------------------------------------------
        % TODO join all cues and solve FW
        fprintf('\t Solving Frank-Wolfe optimization...\n');
        % Prepare inputs for Frank Wolfe (conditional gradient)
        [H_,F,Aeq,Beq,labels] = FW_preamble(N,k,a,m,G,b,Alpha,Zeta,n1,n2);

        % Solve the problem using Frank Wolfe
        [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(H_,F,Aeq,Beq,labels,FW_max_iterations,FW_duality_gap,FW_eps); % minx is the value we want

        optimization_results = reshape(minx,k,[]); optimization_results_percam = cell(2,1);
        % TODO fix this, should work for multiple cameras
        optimization_results_percam{1} = optimization_results(:,1:n1);
        optimization_results_percam{2} = optimization_results(:,n1:end);

        fprintf('\t Found optimal candidates for each target. If debug option, will now show selected candidates.\n');

        % NOTE Show next frame, show previous target location, show all sampled candidates and show best candidate
        if f == debug_test_frames && show_predicted_bbs == true
            for test_cam = 1:length(cameras)
                figure;
                title(['t+1 Candidates (BB) in camera ' cameras{test_cam}]);
                imshow(cameraListImages{test_cam}{start_frames(test_cam)+f});
                hold on;
                %drawCandidateBBs(cell2mat(transpose(cands_percam{test_cam})), 'Green', 'hda', k);
                drawBBs(targs_percam{test_cam}(:,4:7), 'Yellow', 'hda');
                for i=1:size(targs_percam{test_cam})
                    for j=1:k
                        if optimization_results_percam{test_cam}(j,i) == 1
                            % DEBUG
                            rectangle('Position',cands_percam{test_cam}{i}(j,1:4),'EdgeColor', 'Red', 'LineWidth', 1);
                        end
                    end
                end
            end
        end
        if rem(f-1,xi) == 0 && size(targs_in_overlap,1) == length(cameras) % NOTE Only do this if there are targets from all cams
            fprintf('\t Performing inter-camera disambiguation in overlap region...\n');
            fprintf('\t Using a posteriori assignment solving -- Matching...\n');
            % NOTE METHOD ONE OF INTER-CAMERA ASSIGNMENT
            % TODO Build Score matrix (how to generalize this to n cameras? NP-hard assignment problem?)
            n_ov1 = size(targs_in_overlap{1},1);
            n_ov2 = size(targs_in_overlap{2},1);
            [S, A, P, V] = createScoreMatrix(f,n_ov1,n_ov2,targs_in_overlap,images,...
                            d1_metric,d256_metric, motion_models_overlap);

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
                valid_matchings = getValidMatchings(i, S, score_threshold, n_ov2, n_ov1, targs_in_overlap, assignments, valid_matchings);
            end
        end
        %---------------------------------------------------------------------------
        % NOTE homography Correction can be done separately, independently of how the target coupling is solved
        % TODO correct homographies and ALL detections using these homographies
        if rem(f-1,psi) == 0 && ~isempty(valid_matchings{1}) && ~isempty(valid_matchings{2}) && update_homo
            v_matchings = cell(2,1);
            for i=1:length(cameras)
                if ~isempty(valid_matchings{i})
                    v_matchings{i} = cell2mat(transpose(valid_matchings{i}));
                end
            end
            fprintf('\t\t Correcting homographies...\n');
            [H1, H2, cam1_dets_gnd, cam2_dets_gnd, cam1_region_gnd, cam2_region_gnd, n_c1, n_c2] = homography_correction(v_matchings, inplanes, ...
            ground_plane_regions, homog_solver, N_h, rho_r, rho_d, homocorrec_debug);
            homographies{1} = H1; homographies{2} = H2;
            % Update existing camera regions and positions with the new adjusted ones
            ground_plane_regions_adjusted{1} = cam1_region_gnd;
            ground_plane_regions_adjusted{2} = cam2_region_gnd;
            [overlap_adjusted, ~, ~] = computeOverlap(ground_plane_regions_adjusted);
            adjusted_positions{1}{end+1} = cam1_dets_gnd;
            adjusted_positions{2}{end+1} = cam2_dets_gnd;
        end

        %---------------------------------------------------------------------------
        % TODO update motion models using old targets and predicted targets
        fprintf('\t Updating motion models...\n');
        for i=1:N
            for j=1:k
                if optimization_results(j,i) == 1
                    % Update Position
                    motion_models{i}(1:2) = cands{i}(j,5:6);
                    % Update velocity
                    dt = 1.0/fps;
                    motion_models{i}(3:4) = (cands{i}(j,5:6) - targs(i,8:9))/dt;
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
                    tracklets{end+1} = [targs(i,:); targs(i,1:3) cands{i}(j,:)];
                    targs(i,4:9) = cands{i}(j,:);
                end
            end
        end
    end
    if update_homo == false
        % TODO Store these tracklets
        nohomocorrec_tracklets = tracklets;
    end
end
%#############################################################################################################################

% TODO Plot final tracks for debug mostly
%openfig(floor_image); hold on;
figure; hold on;

% TODO Plot ground truths
colors = {'Orange','Purple','Grey'};
colors_adjusted = {'Red','Blue','Black'};
colors_nohomo = {'Pink','SkyBlue'};
gndtruth = cell(2,1);
for i = 1:length(cameras)
    % DEBUG Define a specific frame here for debug to see all candidates of that specific frame
    if show_candidates == true
        if ~isempty(all_candidates{candidates_frame})
            for j=1:size(all_candidates{candidates_frame}{i},2)
                scatter(all_candidates{candidates_frame}{i}{j}(:,5),all_candidates{candidates_frame}{i}{j}(:,6),14,'d','MarkerFaceColor',rgb('White'),'MarkerEdgeColor',rgb(colors_adjusted{i}))
            end
        end
    end
    if draw_regions == true
        drawPoly(ground_plane_regions{i},colors{i},0.5,false); % Draw regions
        if ~isempty(ground_plane_regions_adjusted{i})
            drawPoly(ground_plane_regions_adjusted{i},colors_adjusted{i},0.5,false);
        end
    end
end
if draw_regions == true
    %drawPoly(overlap,colors{3},1.0,false);
    if ~isempty(ground_plane_regions_adjusted{1}) &&  ~isempty(ground_plane_regions_adjusted{2})
        drawPoly(overlap_adjusted, colors_adjusted{3},1.0,false);
    end
end
% TODO Plot no homography correction
nohomocorrec_plots = cell(2,1);
for s = 1:length(nohomocorrec_tracklets)
    if nohomocorrec_tracklets{s}(1,1) == 1
        %plot(nohomocorrec_tracklets{s}(:,8),nohomocorrec_tracklets{s}(:,9),'s-','Color',rgb('Gold'));
        nohomocorrec_plots{1}{end+1} = nohomocorrec_tracklets{s};
    else
        %plot(nohomocorrec_tracklets{s}(:,8),nohomocorrec_tracklets{s}(:,9),'s-','Color',rgb('Silver'));
        nohomocorrec_plots{2}{end+1} = nohomocorrec_tracklets{s};
    end
end
nohomocorrec_plots{1} = cell2mat(transpose(nohomocorrec_plots{1}));
nohomocorrec_plots{2} = cell2mat(transpose(nohomocorrec_plots{2}));
% TODO Plot WITH homography correction
plots = cell(2,1);
for s = 1:length(tracklets)
    if tracklets{s}(1,1) == 1
        %plot(tracklets{s}(:,8),tracklets{s}(:,9),'s-','Color',rgb('Red'));
        plots{1}{end+1} = tracklets{s};
    else
        %plot(tracklets{s}(:,8),tracklets{s}(:,9),'s-','Color',rgb('Blue'));
        plots{2}{end+1} = tracklets{s};
    end
end
plots{1} = cell2mat(transpose(plots{1}));
plots{2} = cell2mat(transpose(plots{2}));

for i = 1:length(cameras)
    nohomocorrec_plots{i} = accumarray(nohomocorrec_plots{i}(:,3),(1:size(nohomocorrec_plots{i},1)).',[],@(x){nohomocorrec_plots{i}(x,:)},{});
    plots{i} = accumarray(plots{i}(:,3),(1:size(plots{i},1)).',[],@(x){plots{i}(x,:)},{});
    for s = 1:size(nohomocorrec_plots{i},1)
        plot(nohomocorrec_plots{i}{s}(:,8),nohomocorrec_plots{i}{s}(:,9),'o--','Color',rgb(colors_nohomo{i}));
        plot(plots{i}{s}(:,8),plots{i}{s}(:,9),'s-','Color',rgb(colors_adjusted{i}));
    end
    for matches = 1:size(valid_matchings{1},1)
        plot([valid_matchings{1}(matches,8); valid_matchings{2}(matches,8)],[valid_matchings{1}(matches,9); valid_matchings{2}(matches,9)],'k');
    end
end

if show_ground_truth == true
    for f = 1:(num_frames - 1)
        if f == debug_gnd_truth_frames + 1 % DEBUG Always draw one more for debug reasons
            break;
        end
        truth1 = gnd_detections{1}{start_frames(1) + f};
        truth2 = gnd_detections{2}{start_frames(2) + f};
        scatter(truth1(:,8),truth1(:,9),'MarkerFaceColor',rgb('Orange'),'MarkerEdgeColor',rgb('Orange'));
        scatter(truth2(:,8),truth2(:,9),'MarkerFaceColor',rgb('Purple'),'MarkerEdgeColor',rgb('Purple'));
    end
end
if draw_regions == true
    legend('Original cam 40 region', 'Corrected cam 40 region', 'Original cam 19 region', 'Corrected cam 19 region', 'Overlap region', ...
    'Pedestrian 1 -- Cam 40','Corrected Pedestrian 1 -- Cam 40', ...
    'Pedestrian 2 -- Cam 40','Corrected Pedestrian 2 -- Cam 40', ...
    'Pedestrian 1 -- Cam 19','Corrected Pedestrian 1 -- Cam 19',...
    'Pedestrian 2 -- Cam 19','Corrected Pedestrian 2 -- Cam 19',...
    'Initial detections Cam 40','Initial detections Cam 19');
else
    legend('Ped 1 Cam 40','Ped 1 Cam 40', ...
    'Ped 2 Cam 40','Ped 2 Cam 40', ...
    'Ped 1 Cam 19','Ped 1 Cam 19',...
    'Ped 2 Cam 19','Ped 2 Cam 19',...
    'Initial detections Cam 40','Initial detections Cam 19');
end
xlabel('x(m)') % x-axis label
ylabel('y(m)') % y-axis label
