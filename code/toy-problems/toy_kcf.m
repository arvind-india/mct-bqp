%1,6882,1,177.69403,134.57555,109.85075,313.00199,-6.90447,-0.65662
%1,6882,2,314.29104,129.80417,99.34328,294.87077,-6.54525,-1.46042
%1,6883,1,177.69403,134.57555,109.85075,313.00199,-6.90447,-0.65662
%1,6883,2,304.73881,122.16998,88.83582,280.55666,-6.18769,-1.43629
%1,6884,1,177.69403,134.57555,103.16418,305.36779,-6.79618,-0.64384
%1,6884,2,285.63433,120.26143,98.38806,270.05964,-5.96631,-1.38292
%1,6885,1,175.78358,125.98708,101.25373,287.23658,-6.38096,-0.65028
%1,6885,2,276.08209,119.30716,100.29851,264.33400,-5.84039,-1.34675
%1,6886,1,175.78358,125.98708,101.25373,287.23658,-6.38096,-0.65028
%1,6886,2,281.81343,114.59127,90.74627,248.05584,-5.40962,-1.41918
%1,6887,1,181.51493,125.98708,95.52239,287.23658,-6.38058,-0.66820
%1,6887,2,282.76866,111.95863,85.01493,244.04236,-5.26176,-1.42828
%1,6888,1,190.11194,121.21571,90.74627,271.96819,-6.02792,-0.72974
%1,6888,2,281.11765,109.80004,73.29280,240.87147,-5.13848,-1.39250

% 2,6812,1,490.91968,34.32239,81.50507,241.85328,-7.14684,-0.54268
% 2,6812,2,371.38781,35.50407,82.75353,234.97163,-6.94853,-1.44299
% 2,6813,1,491.84588,38.02896,86.59913,241.85328,-7.07278,-0.51141
% 2,6813,2,369.26663,34.82514,87.51331,240.15501,-6.85282,-1.42288
% 2,6814,1,492.77207,41.73552,91.69320,241.85328,-7.00026,-0.48080
% 2,6814,2,365.70810,35.43166,92.27310,253.04566,-6.57651,-1.37917
% 2,6815,1,490.91968,44.51544,91.69320,251.11969,-6.74156,-0.48136
% 2,6815,2,354.88716,41.48242,97.03289,263.44103,-6.25307,-1.37616
% 2,6816,1,489.06729,47.29537,91.69320,260.38610,-6.50126,-0.48189
% 2,6816,2,342.79530,41.36304,101.79268,263.56040,-6.23181,-1.43920
% 2,6817,1,483.04703,47.29537,91.69320,262.23938,-6.45251,-0.52183
% 2,6817,2,333.72944,46.28462,106.55246,286.76737,-5.76368,-1.38340
% 2,6818,1,477.02677,47.29537,91.69320,277.06564,-6.17516,-0.54561
% 2,6818,2,328.29479,51.20620,111.31225,297.27112,-5.53660,-1.35268
% 2,6819,1,475.63748,50.53861,94.00868,292.35521,-5.87423,-0.52663
% 2,6819,2,330.51160,49.77617,116.07204,298.70115,-5.54468,-1.32609
% 2,6820,1,474.24819,53.78185,96.32417,289.11197,-5.87378,-0.52810
% 2,6820,2,322.31560,55.75635,117.47632,320.84951,-5.16947,-1.28799

start_frame = '6812.png';
track_frames = {'6813.png' '6814.png' '6815.png' '6816.png' '6817.png'}
%start_frame = '6882.png';
%track_frames = {'6883.png' '6884.png' '6885.png' '6886.png' '6887.png'}
% Target (in start frame)
% Target in cam 40
bb_width = 82.75353;
bb_height = 234.97163;
cx = 371.38781;
cy = 35.50407;
% Target in cam 19
% bb_width = 99.34328;
% bb_height = 294.87077;
% cx = 314.29104;
% cy = 129.80417;

% --- Train
% Get the image patch

original_bb = imcrop(imread(start_frame), [cx cy bb_width bb_height]);
patch_coords = [cx - bb_width cy - bb_height 3*bb_width 3*bb_height];
[x, crop_rect] = imcrop(imread(start_frame), patch_coords);




figure;
subplot(1,2,1);


% Generate the 2d gaussian
original_crop_rect = crop_rect;
sigma = 100;
if crop_rect(1) < 0
    crop_rect(1) = 0;
end
if crop_rect(2) < 0
    crop_rect(2) = 0;
end
% Actual distance from crop
dx = cx - crop_rect(1);
dy = cy - crop_rect(2);

% Gaussian pre-filter outside the pedestrian bb
prefilter_sigma = 10;
Iblur = imgaussfilt(x, prefilter_sigma);
ceildx = cast(dx, 'int32') + 1;
ceilbx = cast(dx, 'int32') + size(original_bb,2);
ceildy = cast(dy, 'int32') + 1;
ceilby = cast(dy, 'int32') + size(original_bb,1);
Iblur((ceildy:ceilby),(ceildx:ceilbx),:) = original_bb(:,:,:);
x = Iblur;
I = imread(start_frame);
I((cast(crop_rect(2), 'int32') + 1:cast(crop_rect(2), 'int32') + size(x,1)),(cast(crop_rect(1), 'int32') + 1:cast(crop_rect(1), 'int32') + size(x,2)),:) = x(:,:,:);
imshow(I);
drawBBs([cx cy bb_width bb_height], 'g', 'hda')
drawBBs(patch_coords, 'w', 'hda')
title(start_frame);

gsize = [size(x,1) size(x,2)];
center = [dx dy];

lambda = 10;
Sig = [sigma 0; 0 sigma];

y = gauss2d(gsize, Sig , center);
learning_rate = 0.1;
tic
alphaf = train(x, y, sigma, lambda);
toc
% Predict on the next frames
for track_frame = track_frames
    track_frame = track_frame{1}
    % Get a patch on the next frame that is in the same place
    z = imcrop(imread(track_frame), patch_coords);
    tic
    responses = detect_patch(alphaf, x, z, sigma);
    toc
    % Get the best possible value according to the KCF tracker
    maxValue= max(responses(:));
    [rowsOfMaxes, colsOfMaxes] = find(responses == maxValue);
    val_x = colsOfMaxes + crop_rect(1);
    val_y = rowsOfMaxes + crop_rect(2);
    % If many similar responses, get best
    val_x = val_x(1);
    val_y = val_y(1);
    tracked_img = imcrop(imread(track_frame), [val_x val_y bb_width bb_height]);
    subplot(1,2,2);
    imshow(imread(track_frame));
    drawBBs([val_x val_y bb_width bb_height], 'r', 'hda')
    drawBBs(patch_coords, 'w', 'hda')
    title(track_frame);

    test_cands = 1;
    dx = 5;
    dy = 10;
    if test_cands == 1
        k = 81;
        % These two can be picked from the KCF patch
        xstep = bb_width/dx;
        ystep = bb_height/dy;
        % Get my k candidates and show them
        % Assign them their weights from the responses vector
        max_resp = -100;
        candidate_responses = zeros(sqrt(k));
        best_cand = [0 0];
        for gridx=-floor(sqrt(k)/2):floor(sqrt(k)/2)
            for gridy=-floor(sqrt(k)/2):floor(sqrt(k)/2)
                k_cx = cx + gridx * xstep + 1;
                k_cy = cy + gridy * ystep + 1;

                % Convert candidate locations in the image to patch locations
                resp_cx = k_cx - crop_rect(1);
                resp_cy = k_cy - crop_rect(2);


                if k_cx > 0 && k_cy > 0

                    candidate_responses(gridy + floor(sqrt(k)/2) + 1, gridx + floor(sqrt(k)/2) + 1) = responses(cast(resp_cy, 'int32'), cast(resp_cx, 'int32'));
                    resp = responses(cast(resp_cy, 'int32'), cast(resp_cx, 'int32'));
                    % Reverse lookup the response for this candidate
                    if resp > max_resp
                        best_cand(1) = k_cx;
                        best_cand(2) = k_cy;
                        max_resp = resp;
                    else
                        rectangle('Position',[k_cx k_cy bb_width bb_height],'EdgeColor', 'b', 'LineWidth', 1);
                    end

                    hold on
                end


            end
        end
        rectangle('Position',[best_cand(1) best_cand(2) bb_width bb_height],'EdgeColor', 'y', 'LineWidth', 1);
    end
    pause(1);
end

function k = kernel_correlation(x1, x2, sigma)
    c = ifft2(sum(conj(fft2(x1)) .* fft2(x2), 3));
    a = x1(:);
    b = x2(:);
    d = (a')*a + (b')*b - 2*c;
    k = exp(-1 / sigma^2 * abs(d) / numel(d));
end

function alphaf = train(x, y, sigma, lambda)
    x = cast(x, 'double');
    x = gpuArray(x);
    k = kernel_correlation(x, x, sigma);
    a = fft2(y) ./ (fft2(k) + lambda);
    alphaf = gather(a);
end

function responses = detect_patch(alphaf, x, z, sigma)
    x = cast(x, 'double');
    z = cast(z, 'double');
    x = gpuArray(x);
    z = gpuArray(z);
    k = kernel_correlation(z, x, sigma);
    r = real(ifft2(alphaf .* fft2(k)));
    responses = gather(r);
end
