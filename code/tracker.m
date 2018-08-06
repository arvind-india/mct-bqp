trackerParams;
dataset = 'hda';
seq = 'hall';
gnd_detections = loadDetections(dataset, cameras, seq); % Load the images (needed for the appearance cues)
cameraListImages = loadImages(image_directories, durations, 0, length(cameras), dataset);
[homographies, invhomographies] = loadHomographies(homography_directory, dataset, cameras);
[inplanes, ground_plane_regions, overlap] = loadPlanes(visibility_regions_directory, homographies, cameras, length(cameras), dataset);

% openfig(floor_image); hold on; % Draw ground plane regions
% colors = {'Red','Blue','Black'};
% for i=1:length(cameras)
%     drawPoly(ground_plane_regions{i},colors{i},0.5,false); % Draw regions
% end
% drawPoly(overlap,colors{end},1.0,false);

%%=========================================================
fprintf('Starting tracking loop:\n');
groups = {};
tracklets = {};

ground_plane_regions_adjusted = cell(2,1);
valid_matchings = cell(2,1);
adjusted_positions = cell(2,1);
weights = [];

for f = 1:(num_frames - 1)
    if f == debug_test_frames + 1
        break; % DEBUG break cycle and force stopping
    end
    %---------------------------------------------------------------------------
    fprintf(['  Frame ' num2str(f) ':\n \t 1.Getting images\n']);
    % Gets f and (f+1) frames for all cameras
    [next_images, images] = getImages(length(cameras), cameraListImages, f, start_frames);
    %---------------------------------------------------------------------------
    fprintf('\t 2.Getting targets from frames in detection frames\n');
    [targs, targs_percam] = getTargets(f, detection_frames, gnd_detections, length(cameras), start_frames);
    n1 = size(targs_percam{1},1);
    n2 = size(targs_percam{2},1);
    N = n1 + n2;
    %---------------------------------------------------------------------------
    % Sample around the targets. We then use these candidates on the next frame
    fprintf('\t 3.Sampling candidates in f+1 frames\n');
    [cands, cands_percam] = sampleCandidates(N, k, length(cameras), targs, homographies, invhomographies, sampling_plane, dx, dy);
    %---------------------------------------------------------------------------
    % NOTE store the ones that are ambiguous for homography correction in targs_in_overlap (i.e gating part 1)
    fprintf('\t 4.Checking for targets in the overlapping regions...\n')
    [targs_in_overlap, n_o, N_o] = getTargetsOverlap(targs, overlap, cameras);
    n1_o = size(targs_in_overlap{1},1);
    n2_o = size(targs_in_overlap{2},1);
    %---------------------------------------------------------------------------
    fprintf('\t 5.Cloning targets...\n')
    cands_clones = getClones(N,length(cameras), k, targs_percam, overlap);
    %---------------------------------------------------------------------------
    fprintf('\t 6.Computing appearance cues (KCF)...\n');
    [a, weights] = appearance(k, targs_percam, cands_percam, cands_clones, images, next_images, sigma, prefilter_sigma, lambda, weights, num_cams, filter, dx, dy);
    fsfsfs;
    %---------------------------------------------------------------------------
    fprintf('\t 7.Computing motion cues...\n');
    % m = motion();
    %---------------------------------------------------------------------------
    fprintf('\t 8.Computing grouping cues (for temporal association only)...\n');
    % G = grouping();
    %---------------------------------------------------------------------------
    fprintf('\t 9.Computing bounding cues (assuming n1=n2 for now)...\n');
    b = bounds(k, n1, cands_percam, cands_clones, ground_plane_regions, penalize_val, reward_val, length(cameras));
    %---------------------------------------------------------------------------
    % NOTE join all cues and solve FW
    fprintf('\t 10.FW optimization! »» \n');
    % Prepare inputs for Frank Wolfe (conditional gradient)
    [H_,F,Aeq,Beq,labels] = FW_preamble(N, k, a, m, G, b, Alpha, Zeta, n1, n2);

    % Solve the problem using Frank Wolfe
    [minx,minf,x_t,f_t,t1_end] = FW_crowd_wrapper(H_, F, Aeq, Beq, labels, FW_max_iterations, FW_duality_gap, FW_eps); % minx is the value we want

    optimization_results = reshape(minx,k,[]);
    optimization_results_percam = cell(2,1);
    opt_results_o = reshape(minx_o,2,[]);

    % TODO fix this, should work for multiple cameras
    optimization_results_percam{1} = optimization_results(:,1:n1);
    optimization_results_percam{2} = optimization_results(:,n1:end);
    opt_results_percam_spatial{1} = opt_results_o(:,1:n1_o);
    opt_results_percam_spatial{2} = opt_results_o(:,n1_o+1:end);
    valid_matchings = getValidMatchings_fw(n_o,cameras,opt_results_percam_spatial,targs_in_overlap,valid_matchings);

    fprintf('\t Found optimal candidates for each target.\n');
    %---------------------------------------------------------------------------
    printf('\t Parsing for spatial association.\n');
    % Parse FW results for this

    fprintf('\t Parsing for spatial association.\n');
    % Parse FW results for this

    %---------------------------------------------------------------------------
    % NOTE homography Correction must be done separately, independently of how the target coupling is solved
    % Correct homographies and ALL detections using these homographies
    [homographies, invhomographies, adjusted_gnd_detections, adjusted_ground_plane_regions, adjusted_overlap] = homography_correction(matchings, inplanes, ground_plane_regions, homog_solver, num_cams);

    %---------------------------------------------------------------------------
    % NOTE update motion models using old targets and predicted targets
end
