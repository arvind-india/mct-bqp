setCaptureParams_campus2;
obj = setupSystemObjects('alameda.mp4', resolutions); % Create System objects used for reading video, detecting moving objects, and displaying the results.
sample = 2000;
show_frame = 871; % Frame(s) to show
detections = cell(sample,1);
% Detect moving objects.
%while ~isDone(obj.reader)
for i=1:sample
    frame = obj.reader.step(); % read a frame
    [centroids, bboxes, mask] = detectObjects(frame, obj); %
    detections{i} = [centroids, bboxes];
    if i == show_frame
        figure
        %waitforbuttonpress;
        title(strcat('Frame: ', num2str(i)));
        hold on;
        imshow(frame);
        drawBBs(detections{i}, rgb('Orange'))
    end
end

function [centroids, bboxes, mask] = detectObjects(frame, obj)
    mask = obj.detector.step(frame); % Detect foreground.
    % Apply morphological operations to remove noise and fill in holes.
    mask = imopen(mask, strel('rectangle', [4, 4]));
    mask = imclose(mask, strel('rectangle', [15, 15]));
    mask = imfill(mask, 'holes');
    [~, centroids, bboxes] = obj.blobAnalyser.step(mask); % Perform blob analysis to find connected components.
end

function obj = setupSystemObjects(video_sequence, resolutions)
    % Create objects for reading a video from a file, drawing the tracked objects in each frame, and playing the video.
    obj.reader = vision.VideoFileReader(video_sequence);  % Create a video file reader.

    % Specify the size and location of the scope window in pixels, as a four-element double vector of the form: [left bottom width height].
    obj.maskPlayer = vision.VideoPlayer('Position', [resolutions(1,1)+40, 768, resolutions(1,1), resolutions(1,2)]); % Video player to display video
    obj.videoPlayer = vision.VideoPlayer('Position', [20, 768, resolutions(1,1), resolutions(1,2)]); % Video player to display the foreground mask.

    % The foreground detector is used to segment moving objects from the background.
    % It outputs a binary mask, where the pixel value of 1 corresponds to the foreground and the value of 0 corresponds to the background.
    obj.detector = vision.ForegroundDetector('NumGaussians', 3, 'NumTrainingFrames', 40, 'MinimumBackgroundRatio', 0.7);

    % Connected groups of foreground pixels are likely to correspond to moving
    % objects.  The blob analysis System object is used to find such groups (called 'blobs' or 'connected components'), and compute their characteristics, such as area, centroid, and the bounding box.
    obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, 'AreaOutputPort', true, 'CentroidOutputPort', true, 'MinimumBlobArea', 200);
end

function drawBBs(dets, bb_color)
    bb_linewidth = 2;
    for k = 1:size(dets,1)
        hold on
        rectangle('Position',dets(k,3:6),'EdgeColor', bb_color, 'LineWidth', bb_linewidth);
    end
end
