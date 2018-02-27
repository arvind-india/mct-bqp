setDetectionParams_campus2;
setTrackerParams;

gnd_detections = load_data();
% Load the images (needed for the appearance cues)
cameraListImages = cell(2,1); inplanes = cell(2,1);
for i=1:length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, 0, i, 'campus2');
    inplanes{i} = dlmread(strcat(regions_folder, cameras{i}, '.txt'));
end
[homographies, invhomographies] = loadHomographies(homography_directory,'campus_2'); % Defined in global variables
ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'campus_2');
[overlap, ~, ~] = computeOverlap(ground_plane_regions);
region_colors = {'Red','Blue','Black'};

%%=========================================================
fprintf('Starting tracking loop:\n');
num_frames = 10; % Number of frames
start = 0;
start_frames = [start start+offset_frames]; % Frames to start collecting images
k = g_candidates ^ 2; % Candidates per target
tau = 3;
groups = {};
N = 0; % Number of targets in all cameras
K = 0; % Number of candidates in all cameras (must be a multiple of k)
for f = 1:(num_frames - 1)
    %---------------------------------------------------------------------------
    fprintf(['Frame ' num2str(f) ':\n']);
    fprintf('\t Getting targets...\n');
    if f == 1
        targs = cell(2,1); % Actual targets from both cameras
    end
    for id = 1:length(cameras)
        targs{id} = gnd_detections{id}{start_frames(id) + f};
    end
    targs = cell2mat(targs');
    targs_percam = (accumarray(targs(:,1),(1:size(targs,1)).',[],@(x){targs(x,:)},{}));
    N = size(targs,1); K = N * k;
    cands = cell(N,1); cands_homo = cands; % Candidates from both cameras for each target, always empty each frame
    cands_percam = cell(2,1); cands_homo_percam = cands_percam;
    %---------------------------------------------------------------------------
    % Sample around the targets. We then use these candidates on the next frame
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
    %---------------------------------------------------------------------------
    % TODO store the ones that are ambiguous for homography correction in targs_in_overlap
    fprintf('\t Checking for targets in the overlapping regions...\n');
    %figure; hold on;
    %for i = 1:length(cameras)
    %    drawPoly(ground_plane_regions{i},region_colors{i},0.5,false);
    %end

    %drawPoly(overlap,region_colors{3},1.0,false);
    targs_in_overlap = {};
    for t = 1:size(targs,1)
        %plot(targs(t,8),targs(t,9),'+','MarkerEdgeColor',region_colors{targs(t,1)},'linewidth',1);
        if polyin([targs(t,8) targs(t,9)],overlap)
            targs_in_overlap{end+1} = targs(t,:);
        end
    end


    %---------------------------------------------------------------------------
    % TODO appearance
    fprintf('\t Computing apperance cues...\n');
    a = cell(length(cameras),1);
    for i = 1:length(cameras)
        n = size(targs_percam{i},1);
        next_image = imread(cameraListImages{i}{start_frames(i)+(f+1)});
        image = imread(cameraListImages{i}{start_frames(i)+(f)});
        [c_a, w, Z, y] = appearance(k,n,targs_percam{i},cands_percam{i},image,next_image,'naive',lambda);
        a{i} = c_a;
        %--------------------------------
        plotAppeance(c_a,i,n,k,cameraListImages,f,targs_percam,cameras,cands_percam, start_frames);
    end
    a = cell2mat(a);
    %---------------------------------------------------------------------------
    % TODO create motion models
    fprintf('\t Creating motion models...\n');
    motion_models = cell(N,1);
    if f == 1
        for t = 1:size(targs,1)
            motion_models{t} = [targs(t,8); targs(t,9); 0; 0];
        end
    end
    %--------------------------------
    % TODO motion
    fprintf('\t Computing motion cues...\n');
    m = cell(length(cameras),1);
    for i = 1:length(cameras)
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

    optimization_results = reshape(minx,k,[]);
    fprintf('\t Found optimal candidates for each target.\n');
    figure; hold on;
    % NOTE Show next frame
    % NOTE Show previous target location
    % NOTE Show all sampled candidates
    % NOTE Show best candidate
    for i=1:N
        for j=1:k
            if optimization_results(i,j) == 1

            end
        end
    end
    kill;
    % TODO disambiguate between targets that are in the overlapping region

    % TODO Build Score matrix

    % TODO Gating

    %---------------------------------------------------------------------------
    % TODO Target Coupling
    fprintf('\t Target Coupling/Disambiguation...\n');

    % TODO correct homographies and ALL detections using these homographies
    %---------------------------------------------------------------------------

    %---------------------------------------------------------------------------
    % TODO update motion models using old targets and predicted targets
    %---------------------------------------------------------------------------

    % TODO clear targets, use the predicted ones as new targets
    %---------------------------------------------------------------------------
    kill;
end
