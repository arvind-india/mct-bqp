set(0,'DefaultFigureVisible','on');
setDetectionParams_hda_hall; setTrackerParams;

gnd_detections = load_data('hda', cameras); % Load the images (needed for the appearance cues)
cameraListImages = cell(2,1); inplanes = cell(2,1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, durations(i), 0, 'hda');
    inplanes{i} = load(strcat(visibility_regions_directory,num2str(cameras{i}),'.mat')); inplanes{i} = inplanes{i}.t;
end
[homographies, invhomographies] = loadHomographies(homography_directory,'hda', cameras);
ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'hda');
[overlap, ~, ~] = computeOverlap(ground_plane_regions);
%%=========================================================
num_frames = 7; % Number of frames
start = 6811;
start_frames = [start + offset_frames start]; % Frames to start collecting images
g_candidates = 9; % basis for the number of candidates
k = g_candidates ^ 2; % Candidates per target
tau = 1; h_tau = 1; psi = 8; xi = 8;
N = 0; % Number of targets in all cameras
penalize_val = 1;
reward_val = -1;

gating_distance = 6.0; % This does actually correspond to 6 meters
score_threshold = 0.4;

initial_speed_x = 10.0;
initial_speed_y = 0;
sampling_plane = 'ground'; % NOTE Must be either 'camera' or 'ground'
dx = 10; dy = 50; % Sampling in the camera space
g_dx = 0.05; g_dy = 0.02; % Sampling in the

clustering = 'single';
comfort_distance = 1.5; % Anyone closer than this meters is in talking range?
FW_max_iterations = 5000;  % max number of iteration, I found out the opts.TOL (tolerance?) is more important in the convergence in my problem
FW_duality_gap = 2; % This sets the duality gap and you can change it if you want to change the tolerance for FW convergence
FW_eps = 1e-12; % (eps in matlab = 1e-16) not sure why this is needed
lambda = 0.5; % variable for the appearance cues
Alpha = [1.0 1.0]; % weight of the appearance constraint
Zeta = [1.0 1.0]; % weight of the motion constraint
a_sigma{1} = [1 ^ 2 0.0; 0.0 1 ^ 2]; a_sigma{2} = [1 ^ 2 0.0; 0.0 1 ^ 2];
m_sigma{1} = [0.5 0; 0 0.5]; m_sigma{2} = [0.5 0; 0 0.5];
G_sigma =  2 * (2 ^ 2);

h_group_distance = 0.5; % Estimate for the distance of groups made by homographies
h_FW_max_iterations = 5000;
h_FW_duality_gap = 2;
h_FW_eps = 1e-12;
h_lambda = 0.5;
h_Alpha = [1.0 1.0];
h_Zeta = [1.0 1.0];
h_a_sigma{1} = [1 ^ 2 0.0; 0.0 1 ^ 2]; h_a_sigma{2} = [1 ^ 2 0.0; 0.0 1 ^ 2];
h_m_sigma{1} = [0.5 0; 0 0.5]; h_m_sigma{2} = [0.5 0; 0 0.5];
h_G_sigma =  2 * (2 ^ 2);

appearance_method = 'naive'; % NOTE Must be either 'naive' or 'fourier'
intercam_method = 'frank-wolfe'; % NOTE Must be either 'lapjv' or 'frank-wolfe'
homog_solver = 'svd'; % Method to compute homographies, NOTE must be either 'svd' or 'ransac'
filter = 'none'; % NOTE must be either 'recursive' or 'non-recursive' or 'none' (kalman in the future?)
weights = cell(2,1); weights_o = cell(2,1);
%%=========================================================
debug_test_frames = 1; % DEBUG test these frames
homo_toggle = 1; % DEBUG Set this to 1 if you wish to only run with homography correction!
show_ground_truth = true;
show_candidates = true;
show_predicted_bbs = false;
draw_regions = true;
homocorrec_debug = 'no_debug';
candidates_frame = 2;
debug_gnd_truth_frames = 3;
nohomocorrec_tracklets = {};
%%=========================================================
% Frames at which detections are available/allowed
detection_frames = [1; 4; 9]; % These are used to updated targets/number of targets at these frames
%%=========================================================
fprintf('Starting tracking loop:\n');
for update_homo = homo_toggle:1 % DEBUG merely for debug, would never use this is "production"
    fprintf(['Updating homography set to ' num2str(update_homo) '\n']);
    all_candidates = cell(num_frames,1); % Store all candidates (in gnd plane)
    ground_plane_regions_adjusted = cell(2,1);
    valid_matchings = cell(2,1);
    groups = {}; adjusted_positions = cell(2,1); tracklets = {};

    for f = 1:(num_frames - 1)
        if f == debug_test_frames+1
            break;
        end
        fprintf(['  Frame ' num2str(f) ':\n \t 1.Getting images...\n']);
        next_images = cell(length(cameras),1); images = cell(length(cameras),1);
        for i = 1:length(cameras)
            try
                next_images{i} = imread(cameraListImages{i}{start_frames(i)+(f+1)});
                images{i} = imread(cameraListImages{i}{start_frames(i)+(f)});
            catch ME
                fprintf('Could not find dataset. If stored in a HDD, make sure it is mounted.\n');
            end
        end
        %---------------------------------------------------------------------------
        % TODO Detection frames should be used here
        fprintf('\t 2.Getting targets from frames in detection frames...\n');
        if ismember(f,detection_frames)
            targs = cell(2,1); % Targets from all cameras
            for id = 1:length(cameras)
                targs{id} = gnd_detections{id}{start_frames(id) + f};
            end
            targs = cell2mat(targs); % For some reason this work on one computer and on the other one does not
            % NOTE If the targets are empty on the first frame, can't track
            if isempty(targs)
                error('Cannot possibly track if there are no detections on the first given frame');
            end
        end

        targs_percam = (accumarray(targs(:,1),(1:size(targs,1)).',[],@(x){targs(x,:)},{}));
        N = size(targs,1);
        n1 = size(targs_percam{1},1); n2 = size(targs_percam{2},1);
        cands = cell(N,1); cands_percam = cell(2,1); % Candidates from both cameras for each target, always empty each frame
        %---------------------------------------------------------------------------
        % Sample around the targets. We then use these candidates on the next frame
        fprintf('\t 3.Sampling candidates...\n');
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
                    sampling_dx = g_dx; % TODO Bernardino proposed a way to actually get bb width from gnd plane
                    sampling_dy = g_dy; % TODO but we would need some data we do not have :(
                    %t_pos = transpose(H(homographies{targs(t,1)},[t_rect(3)/2; t_rect(4)]));
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
        % check for potential new targets to add from the detections
        fprintf('\t 4.Checking for potential new targets...\n');
        % Check if detections are available. If there are more detections than currently there are targets consider this
        if f ~= 1
            if size(gnd_detections{id}{start_frames(id) + f},1) > size(targs_percam{id},1)
                fprintf('\t \t More detections than targets! \n');
            end
        end
        %---------------------------------------------------------------------------
        % NOTE store the ones that are ambiguous for homography correction in targs_in_overlap (i.e gating part 1)
        fprintf('\t 5.Checking for targets in the overlapping regions...\n')
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
        fprintf('\t --------------------------------- \n')
        %---------------------------------------------------------------------------
        % NOTE appearance
        fprintf('\t 6.Computing appearance cues...\n');
        a = cell(length(cameras),1);
        for i = 1:length(cameras)
            n = size(targs_percam{i},1);
            % TODO implement fourier method to do this following
            [c_a, w, Z_app, y] = appearance(k,n,targs_percam{i},cands_percam{i},images{i},...
                next_images{i},appearance_method,lambda,a_sigma{i},dx,dy, g_candidates, weights{i},filter);
            a{i} = c_a;
            %plotAppearanceBBs(i,n,k,cameraListImages,f,targs_percam,cameras,cands_percam, start_frames);
            %plotAppearanceValues(i,n,k,c_a,cands_homo_percam,cameras);
            %plotAppearanceWeighs(i,n,k,w,weights{i}); % w are the new weights and weights{i} are the previous ones

            % TODO Store weights and use them in a filter-like implementation
            weights{i} = w;
        end
        a = cell2mat(a);
        % Normalize for each candidate
        a = reshape(a,k,N);
        for t = 1:size(a,2)
            a(:,t) = a(:,t)./abs(min(a(:,t)));
        end
        a = reshape(a,k*N,1);
        %---------------------------------------------------------------------------
        % NOTE create motion models
        if ismember(f,detection_frames)
            fprintf('\t 7.Creating motion models...\n');
            motion_models = cell(N,1);
            for t = 1:size(targs,1)
                motion_models{t} = [targs(t,8); targs(t,9); initial_speed_x; initial_speed_y];
            end
        end
        %--------------------------------
        % NOTE motion
        fprintf('\t 8.Computing motion cues...\n');
        m = cell(length(cameras),1);
        for i = 1:length(cameras)
            n = size(targs_percam{i},1);
            c_m = motion(n,k,motion_models,cands_percam{i},fps,m_sigma{i});
            m{i} = c_m;
            %plotMotion(i, c_m, k, n, floor_image, cands_percam);
        end
        m = cell2mat(m);
        % Normalize for each candidate
        m = reshape(m,k,N);
        for t = 1:size(m,2)
            m(:,t) = m(:,t)./abs(min(m(:,t)));
        end
        m = reshape(m,k*N,1);

        %--------------------------------
        % NOTE create groups
        fprintf('\t 9.Creating groups...\n');
        if rem(f,tau) == 0
            groups = cell(2,1);
            for i = 1:length(cameras)
                    Y = pdist(targs_percam{i}(:,8:9));
                    Z = linkage(Y,clustering);
                    C = cluster(Z,'cutoff',comfort_distance,'criterion','distance');
                    groups{i} = C;
            end
        end
        %--------------------------------
        % NOTE grouping
        fprintf('\t 10.Computing grouping cues...\n');
        T = grouping(N,k,groups,targs,targs_percam,cands,G_sigma);
        Dinvsq = diag(sum(T,2)).^(-1/2); %row sum
        Dinvsq(~isfinite(Dinvsq)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
        G = eye(N*k) - Dinvsq*T*Dinvsq; %Normalized Laplacian matrix so G is convex
        %plotGrouping(T,G);
        G = zeros(N*k);
        %--------------------------------
        % NOTE out-of-bounds cue
        b = cell(length(cameras),1);
        for i = 1:length(cameras)
            n = size(targs_percam{i},1);
            c_b = bounds(i,k,n,cands_percam,ground_plane_regions,penalize_val,reward_val);
            %plotBounds(i, c_m, k, n, floor_image, cands);
            b{i} = c_b;
        end
        b = cell2mat(b);
        %---------------------------------------------------------------------------
        % NOTE join all cues and solve FW
        fprintf('\t «« (TEMPORAL) Using FW optimization! »» \n');
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
            fprintf('\t 11. Performing inter-camera disambiguation in overlap region...\n');
            n1_o = size(targs_in_overlap{1},1);
            n2_o = size(targs_in_overlap{2},1);
            n_o = {n1_o, n2_o};
            N_o = n1_o + n2_o;
            if n1_o == n2_o
                fprintf('\t \t Same number of targets in overlap from all cams.\n');
            else
                fprintf('\t \t Different number of targets in overlap from all cams.\n');
            end
            %---------------------------------------------------------------------------
            if strcmp(intercam_method,'lapjv')
                fprintf('\t «« (SPATIAL) Using LAP-JV matching! »» \n');
                %---------------------------------------------------------------------------
                % TODO Build Score matrix (how to generalize this to n cameras? NP-hard assignment problem?)
                [S, A, P, V] = createScoreMatrix(f,n1_o,n2_o,targs_in_overlap,images,d1_metric,d256_metric, motion_models_overlap);

                % NOTE Gating (i.e gating part2)
                for a1 = 1:n1_o
                    for b1 = 1:n2_o
                        if P(a1,b1) > gating_distance
                            S(a1,b1) = Inf;
                        end
                    end
                end
                % NOTE Target Coupling (disambiguate between targets that are in the overlapping region)
                assignments = lapjv(S,eps); % NOTE eps can be changed to accelerate the algorithm
                for i=1:size(S,2)
                    valid_matchings = getValidMatchings_lapjv(i, S, score_threshold, n2_o, n1_o, targs_in_overlap, assignments, valid_matchings);
                end
            elseif strcmp(intercam_method,'frank-wolfe')
                fprintf('\t «« (SPATIAL) Using FW optimization! »» \n');
                %---------------------------------------------------------------------------
                % NOTE Appearance and motion constraint
                a_o = cell(length(cameras),1);
                m_o = cell(length(cameras),1);
                for i = 1:length(cameras)
                    cands_o = cell(n_o{i},1);
                    o_c = rem(i,2) + 1; % TODO This is a dirty hack
                    targs_o = targs_in_overlap{i};
                    c_o = targs_in_overlap{o_c};
                    for j = 1:n_o{i}
                        cands_o{j} = c_o(:,4:9);
                    end

                    [c_a_o, w_o, Z_o, y_o] = appearance(n_o{o_c},n_o{i},targs_o,cands_o,images{i},...
                        images{o_c},appearance_method,h_lambda,h_a_sigma{i},dx,dy,g_candidates, weights_o{i},filter);
                    a_o{i} = c_a_o;
                    weights_o{i} = w_o; % TODO Store weights which might be useful
                    c_m_o = motion(n_o{i},n_o{o_c},motion_models,cands_o,fps,h_m_sigma{i});
                    m_o{i} = c_m_o;
                end
                a_o = cell2mat(a_o); m_o = cell2mat(m_o);
                %---------------------------------------------------------------------------
                % NOTE Intercam groups
                targs_o = cell2mat(targs_in_overlap);
                % Create these "intercam" groups which should reflect the "same" pedestrian /encourage different cams)
                if rem(f,h_tau) == 0
                    Y_o = pdist(targs_o(:,8:9));
                    Z_o = linkage(Y_o,clustering); % Use euclidean distances to do hierarchical clustering (single is default)
                    C_o = cluster(Z_o,'cutoff',h_group_distance,'criterion','distance');
                    groups_o = C_o;
                    figure; dendrogram(Z_o);
                end
                %--------------------------------
                % NOTE grouping
                T_o = h_grouping(N_o,targs_o,targs_in_overlap,groups_o,h_G_sigma);
                Dinvsq_o = diag(sum(T_o,2)).^(-1/2); %row sum
                Dinvsq_o(~isfinite(Dinvsq_o)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
                G_o = Dinvsq_o*T_o*Dinvsq_o; %TODO Due to its structure it cant be Normalized Laplacian matrix so G is convex
                %G_o = zeros(N_o*2,N_o*2);
                %---------------------------------------------------------------------------
                % NOTE Solve the optimization problem
                b_o = zeros(length(a_o),1); % NOTE This is unecessary in the spatial domain
                [H_o,F_o,Aeq_o,Beq_o,labels_o] = FW_preamble(N_o,2,a_o,m_o,G_o,b_o,h_Alpha,h_Zeta,n1_o,n2_o);
                % Solve the problem using Frank Wolfe
                [minx_o,minf_o,x_t_o,f_t_o,t1_end_o] = FW_crowd_wrapper(H_o,F_o,Aeq_o,Beq_o,labels_o,h_FW_max_iterations,h_FW_duality_gap,h_FW_eps);

                opt_results_o = reshape(minx_o,2,[]);
                % TODO fix this, should work for multiple cameras
                opt_results_percam_o{1} = opt_results_o(:,1:n1_o);
                opt_results_percam_o{2} = opt_results_o(:,n1_o+1:end);

                % TODO Process matches, initially it's an identity matrix that shows a bias towards
                % targets with the same number
                valid_matchings = getValidMatchings_fw(n_o,cameras,opt_results_percam_o,targs_in_overlap,valid_matchings);
            end
        end
        %---------------------------------------------------------------------------
        % NOTE homography Correction can be done separately, independently of how the target coupling is solved
        % Correct homographies and ALL detections using these homographies
        if rem(f-1,psi) == 0 && ~isempty(valid_matchings{1}) && ~isempty(valid_matchings{2}) && update_homo
            v_matchings = cell(2,1);
            for i=1:length(cameras)
                if ~isempty(valid_matchings{i})
                    v_matchings{i} = cell2mat(transpose(valid_matchings{i}));
                end
            end
            fprintf('\t\t Correcting homographies...\n');
            [rho_r, rho_d, best_N] = determineRho(v_matchings, inplanes, ground_plane_regions, homog_solver); % Determines good rho values for convergence
            [Hs, cam_dets_gnd, cam_region_gnd, n_c] = homography_correction(v_matchings, inplanes, ...
            ground_plane_regions, homog_solver, best_N, rho_r, rho_d, homocorrec_debug);
            homographies = Hs;
            for i=1:length(cameras)
                invhomographies{i} = inv(homographies{i});
                % Update existing camera regions and positions with the new adjusted ones
                ground_plane_regions_adjusted{i} = cam_region_gnd{i};
                % TODO compute overlap inside the homography_correction func
                adjusted_positions{i}{end+1} = cam_dets_gnd{i};
            end
            [overlap_adjusted, ~, ~] = computeOverlap(ground_plane_regions_adjusted);
        end
        %---------------------------------------------------------------------------
        % NOTE update motion models using old targets and predicted targets
        fprintf('\t Updating motion models...\n');
        for i=1:N
            for j=1:k
                if optimization_results(j,i) == 1
                    % Update Position
                    motion_models{i}(1:2) = cands{i}(j,5:6);
                    % Update Speed
                    motion_models{i}(3:4) = (cands{i}(j,5:6) - targs(i,8:9))/dt; % dt is 1/fps
                end
            end
        end
        %---------------------------------------------------------------------------
        fprintf('\t Saving results, using results as next targets...\n');
        % TODO Store target <-> candidate pairs so we can plot them later
        % TODO "clear" targets, use the predicted ones as new targets
        % TODO get size of tracklets to initialize it
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
        % NOTE Store these tracklets purely for debug
        nohomocorrec_tracklets = tracklets;
    end
end

% NOTE Plot final tracks for debug mostly
plot_output(all_candidates, ground_plane_regions, ground_plane_regions_adjusted, cameras, show_candidates, ...
show_ground_truth, candidates_frame, draw_regions, tracklets, nohomocorrec_tracklets, gnd_detections, overlap_adjusted, ...
num_frames,debug_gnd_truth_frames,start_frames);
