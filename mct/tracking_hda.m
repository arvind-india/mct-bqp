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

%%=========================================================
fprintf('Starting tracking loop:\n');
num_frames = 10; % Number of frames
start = 0;
start_frames = [start start + offset_frames]; % Frames to start collecting images
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
    cands = cell(N,1); cands_homo = cands; % Candidates from both cameras for each target, always empty each frame
    cands_percam = cell(2,1); cands_homo_percam = cands_percam;

end
