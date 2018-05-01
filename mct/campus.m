set(0,'DefaultFigureVisible','on');
setDetectionParams_campus2;
setTrackerParams;

gnd_detections = load_data('mini-campus2', cameras);






% Load the images (needed for the appearance cues)
cameraListImages = cell(2,1); inplanes = cell(2,1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, 0, i, 'campus2');
    inplanes{i} = dlmread(strcat(regions_folder, cameras{i}, '.txt'));
end
[homographies, invhomographies] = loadHomographies(homography_directory,'campus_2', cameras); % Defined in global variables
ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'campus_2');
[overlap, ~, ~] = computeOverlap(ground_plane_regions);

% TODO Plot mini-campus for debug
figure; hold on;
colors = {'Red','Blue','Black'};
for i=1:length(cameras)
    drawPoly(ground_plane_regions{i},colors{i},0.5,false); % Draw regions
end
drawPoly(overlap,colors{3},0.5,false);
for j=1:size(gnd_detections{i},1)
    for i=1:length(cameras)
        if ~isempty(gnd_detections{i}{j})
            if i == 1
                scatter(gnd_detections{i}{j}(:,8),gnd_detections{i}{j}(:,9),8,'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red'));
            end
            if i == 2
                scatter(gnd_detections{i}{j}(:,8),gnd_detections{i}{j}(:,9),8,'MarkerFaceColor',rgb('Blue'),'MarkerEdgeColor',rgb('Blue'));
            end
        end
    end
    %waitforbuttonpress;
end

%%=========================================================
fprintf('Starting tracking loop:\n');
num_frames = 10; % Number of frames
start = 0;
start_frames = [start start]; % Frames to start collecting images, here in the mini-campus they are already prealigned
g_candidates = 5;
k = g_candidates ^ 2; % Candidates per target
tau = 3;
groups = {};
N = 0; % Number of targets in all cameras
for f = 1:(num_frames - 1)
    next_images = cell(length(cameras),1); images = cell(length(cameras),1);
    for i = 1:length(cameras)
        next_images{i} = imread(cameraListImages{i}{start_frames(i)+(f+1)});
        images{i} = imread(cameraListImages{i}{start_frames(i)+(f)});
    end
    %---------------------------------------------------------------------------
    fprintf(['Frame ' num2str(f) ':\n']);
    fprintf('\t Getting targets...\n');
    if f == 1
        targs = cell(length(cameras),1); % TODO Actual targets from both cameras
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
        t_rect = targs(t,4:7);
        cands{t} = zeros(k,4);
        counter = 1;
        for gridx=-2:2
          for gridy=-2:2
            cx = t_rect(1) + gridx * t_rect(3)/10; % startx + gridx * xstep
            cy = t_rect(2) + gridy * t_rect(4)/10; % starty + gridy * ystep
            % TODO Compute homography transformation of candidates and store them in cand_homo
            cands_homo{t}(counter,:) = transpose(H_alt(homographies{targs(t,1)}, [cx+t_rect(3)/2 cy+t_rect(4)]));
            cands{t}(counter,:) = [cx cy t_rect(3) t_rect(4)];
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
    for t = 1:size(targs,1)
        if polyin([targs(t,8) targs(t,9)],overlap)
            targs_in_overlap{end+1} = targs(t,:);
        end
    end
    targs_in_overlap = cell2mat(targs_in_overlap');
    targs_in_overlap = (accumarray(targs_in_overlap(:,1),(1:size(targs_in_overlap,1)).',[],@(x){targs_in_overlap(x,:)},{}));
    %---------------------------------------------------------------------------
    % TODO appearance
    fprintf('\t Computing apperance cues...\n');
    a = cell(length(cameras),1);
    for i = 1:length(cameras)
        n = size(targs_percam{i},1);
        [c_a, w, Z, y] = appearance(k,n,targs_percam{i},cands_percam{i},images{i},next_images{i},'naive',lambda);
        a{i} = c_a;
        %--------------------------------
        %plotAppeance(c_a,i,n,k,cameraListImages,f,targs_percam,cameras,cands_percam, start_frames);
    end
    a = cell2mat(a);
    %---------------------------------------------------------------------------
    % TODO create motion models
    fprintf('\t Creating motion models...\n');
    motion_models = cell(N,1);
    if f == 1
        for t = 1:size(targs,1)
            motion_models{t} = [targs(t,8); targs(t,9); 50; 0];
        end
    end
    %--------------------------------
    % TODO motion
    fprintf('\t Computing motion cues...\n');
    m = cell(length(cameras),1);
    for i = 1:length(cameras)
        n = size(targs_percam{i},1);
        c_m = motion(n,k,motion_models,cands_homo_percam{i},fps);
        m{i} = c_m;
        %plotMotion();
    end
    m = cell2mat(m);
    %---------------------------------------------------------------------------
    % TODO create groups
    fprintf('\t Creating groups...\n');
    groups = cell(2,1);
    comfort_distance = 5; % Anyone closer than this meters is in talking range?
    if f == 1
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
    T = grouping(N,k,groups,targs,targs_percam,cands_homo);
    Dinvsq = diag(sum(T,2)).^(-1/2); %row sum
    Dinvsq(~isfinite(Dinvsq)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
    G = eye(N*k) - Dinvsq*T*Dinvsq; %Normalized Laplacian matrix so G is convex
    %plotGrouping();

    %---------------------------------------------------------------------------
    % TODO join all cues and solve FW
    fprintf('\t Solving Frank-Wolfe optimization...\n');
    % Prepare inputs for Frank Wolfe (conditional gradient)
    [H_,F,Aeq,Beq,labels] = FW_preamble(N,k,a,m,G);

    % Solve the problem using Frank Wolfe
    [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(H_,F,Aeq,Beq,labels); % minx is the value we want

    optimization_results = reshape(minx,k,[]); optimization_results_percam = cell(2,1);
    % TODO fix this, should work for multiple cameras
    optimization_results_percam{1} = optimization_results(:,1:n1);
    optimization_results_percam{2} = optimization_results(:,n1:end);

    fprintf('\t Found optimal candidates for each target. If debug option, will now show selected candidates.\n');

    % NOTE Show next frame
    % NOTE Show previous target location
    % NOTE Show all sampled candidates
    % NOTE Show best candidate
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
    if ~isempty(targs_in_overlap)
        % NOTE METHOD ONE OF INTER-CAMERA ASSIGNMENT
        % TODO Build Score matrix (how to generalize this to n cameras? NP-hard assignment problem?)
        n_ov1 = size(targs_in_overlap{1},1);
        n_ov2 = size(targs_in_overlap{2},1);
        [S, A, P, V] = createScoreMatrix(n_ov1,n_ov2,targs_in_overlap,images,d1_metric,d256_metric);
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
        % NOTE METHOD TWO OF INTER-CAMERA ASSIGNMENT

        %---------------------------------------------------------------------------
        % NOTE homography Correction can be done separately, independently of how the target coupling is solved
        % TODO correct homographies and ALL detections using these homographies
    end
    %#############################################################################################################################
    %---------------------------------------------------------------------------
    % TODO update motion models using old targets and predicted targets
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
    % TODO Store trajectories

    %---------------------------------------------------------------------------
    % TODO "clear" targets, use the predicted ones as new targets
    for i=1:N
        for j=1:k
            if optimization_results(j,i) == 1
                targs(i,4:7) = cands{i}(j,:);
                targs(i,8:9) = cands_homo{i}(j,:);
            end
        end
    end
end
