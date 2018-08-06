function [a, weights] = appearance(k, targs_percam, cands_percam, cands_clones, current_frame, next_frame, sigma, prefilter_sigma, lambda, weights, num_cams, filter, dx, dy)
    a = cell(num_cams,1);
    for c = 1:num_cams
        c_a = {};
        for i = 1:size(targs_percam{c},1)
            fprintf('\t 6.1. Processing "normal" targets for temporal association (KCF)...\n');
            % Get target
            target = targs_percam{c}(i,4:7);
            cx = target(1);
            cy = target(2);
            bb_width = target(3);
            bb_height = target(4);

            % Get target candidates
            % candidates = cands_percam{c}{i}(:,1:4);

            % Get BB patch and training patch
            original_bb = imcrop(current_frame{c}, target);
            patch_coords = [cx - bb_width cy - bb_height 3 * bb_width 3 * bb_height];
            [x, crop_rect] = imcrop(next_frame{c}, patch_coords);
            if crop_rect(1) < 0
                crop_rect(1) = 0;
            end
            if crop_rect(2) < 0
                crop_rect(2) = 0;
            end

            % Gaussian pre-filter
            dx = cx - crop_rect(1);
            dy = cy - crop_rect(2);
            Iblur = imgaussfilt(x, prefilter_sigma);
            ceildx = cast(dx, 'int32') + 1;
            ceilbx = cast(dx, 'int32') + size(original_bb,2);
            ceildy = cast(dy, 'int32') + 1;
            ceilby = cast(dy, 'int32') + size(original_bb,1);
            Iblur((ceildy:ceilby),(ceildx:ceilbx),:) = original_bb(:,:,:);
            x = Iblur;

            % Generate gaussian labels y
            gsize = [size(x,1) size(x,2)];
            center = [dx dy];
            Sig = [sigma 0; 0 sigma]; % Circular covariance
            y = gauss2d(gsize, Sig , center);

            % Train
            alphaf = train(x, y, sigma, lambda);

            % Get candidate weights
            z = imcrop(next_frame{c}, patch_coords);
            responses = detect_patch(alphaf, x, z, sigma);

            xstep = bb_width/dx;
            ystep = bb_height/dy;
            candidate_responses = zeros(k);
            t = 1;
            for gridx=-floor(sqrt(k)/2):floor(sqrt(k)/2)
                for gridy=-floor(sqrt(k)/2):floor(sqrt(k)/2)
                    k_cx = cx + gridx * xstep + 1;
                    k_cy = cy + gridy * ystep + 1;

                    % Convert candidate locations in the image to patch locations
                    resp_cx = k_cx - crop_rect(1);
                    resp_cy = k_cy - crop_rect(2);
                    if k_cx > 0 && k_cy > 0
                        % candidate_responses(gridy + floor(sqrt(k)/2) + 1, gridx + floor(sqrt(k)/2) + 1) = responses(cast(resp_cy, 'int32'), cast(resp_cx, 'int32'));
                        candidate_responses(t) = responses(cast(resp_cy, 'int32'), cast(resp_cx, 'int32'));
                    end
                    t = t + 1;
                end
            end
            c_a{end+1} = -candidate_responses;


            fprintf('\t 6.1. Processing "clone" targets for spatial association (KCF)...\n');

            % Get a patch for this candidate in the frame of the other image (prev)
            candidates_clones = cands_clones{c}{i}(:,1:4);
            clone_responses = zeros(k);
            for j=1:k
                if candidates_clones(j,1) ~= 0
                    ccx = candidates_clones(j,1);
                    ccy = candidates_clones(j,2);
                    cbb_width = candidates_clones(j,3);
                    cbb_height = candidates_clones(j,4);
                    z = imcrop(current_frame{getOtherCamIdx(c)}, [ccx - cbb_width ccy - cbb_height 3 * cbb_width 3 * cbb_height]);
                    z_resized = imresize(z, [size(x,1) size(x,2)]);
                    % Resize the patch to be of the same size as the patch that this was trained on?
                    % TODO Alternative is to resize the actual weights OR to train a filter on the candidates and resize to the original one
                    spatial_responses = detect_patch(alphaf, x, z_resized, sigma);
                    spatial_responses = imresize(spatial_responses, [size(z,1) size(z,2)]);
                    %clone_responses
                end
            end

            % TODO Recursive filter (use weights)







        end
        a{c} = cell2mat(c_a);
    end

end

function oi = getOtherCamIdx(idx)
  oi = rem(idx,2) + 1; % TODO This is a dirty hack to get 1 if 2 or 2 if 1
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

%Image descriptor based on Histogram of Orientated Gradients for gray-level images. This code
%was developed for the work: O. Ludwig, D. Delgado, V. Goncalves, and U. Nunes, 'Trainable
%Classifier-Fusion Schemes: An Application To Pedestrian Detection,' In: 12th International IEEE
%Conference On Intelligent Transportation Systems, 2009, St. Louis, 2009. V. 1. P. 432-437. In
%case of publication with this code, please cite the paper above.
function h = HOG(Im)
    nwin_x = 3;%set here the number of HOG windows per bound box
    nwin_y = 3;
    B=9;%set here the number of histogram bins
    [L,C]=size(Im); % L num of lines ; C num of columns
    H=zeros(nwin_x*nwin_y*B,1); % column vector with zeros
    m=sqrt(L/2);
    if C==1 % if num of columns==1
        Im=im_recover(Im,m,2*m);%verify the size of image, e.g. 25x50
        L=2*m;
        C=m;
    end
    Im=double(Im);
    step_x=floor(C/(nwin_x+1));
    step_y=floor(L/(nwin_y+1));
    cont=0;
    hx = [-1,0,1];
    hy = -hx';
    grad_xr = imfilter(double(Im),hx);
    grad_yu = imfilter(double(Im),hy);
    angles=atan2(grad_yu,grad_xr);
    magnit=((grad_yu.^2)+(grad_xr.^2)).^.5;
    for n=0:nwin_y-1
        for m=0:nwin_x-1
            cont=cont+1;
            angles2=angles(n*step_y+1:(n+2)*step_y,m*step_x+1:(m+2)*step_x);
            magnit2=magnit(n*step_y+1:(n+2)*step_y,m*step_x+1:(m+2)*step_x);
            v_angles=angles2(:);
            v_magnit=magnit2(:);
            K=max(size(v_angles));
            %assembling the histogram with 9 bins (range of 20 degrees per bin)
            bin=0;
            H2=zeros(B,1);
            for ang_lim=-pi+2*pi/B:2*pi/B:pi
                bin=bin+1;
                for k=1:K
                    if v_angles(k)<ang_lim
                        v_angles(k)=100;
                        H2(bin)=H2(bin)+v_magnit(k);
                    end
                end
            end

            H2=H2/(norm(H2)+0.01);
            H((cont-1)*B+1:cont*B,1)=H2;
        end
    end
end

function BVTfeature = extractBVT(paddedImage,maskset)
    % Extracts BVT histograms from the given images and parts.

    [nParts] = size(maskset,2);

    % Resizing maskset from 128x64 to image size
    [hei,wid,chs] = size(paddedImage);
    [heiM, widM] = size(maskset{1});
    if max([heiM, widM] ~= [hei, wid])
        assert(heiM==128 && widM==64, ' Something''s wrong, body-part masks are not same size as padded image, nor are they 128x64 size.')
        %warning(['Body-part masks of original size do not exist, probably because they would be too large (several GB).' ...
        %    ' Using sub-sampled masks (128x64) instead.'])
        % and Resizing masks to original image size
        clear thisMaskSet,
        for partIt = 1:nParts
            resizedMaskSet{partIt} = imresize(maskset{partIt},[hei,wid]);
        end
    else
        resizedMaskSet = maskset;
    end

    % Default histogram feature paramethers
    % display('TODO: check if part weights improves results')
    parVT = struct('NBINs', [10 10 10],'quantMinval',1,'scaleV',36,'scaleB',232,...
        'partWeights', [0.44 0.42 0.42 0.44 0.06 1]); %leg thigh thigh leg head body
    weights = parVT.partWeights;


    scaleB = parVT.scaleB / 100;
    scaleV = parVT.scaleV / 100;
    weights = parVT.partWeights;
    qlevels = parVT.NBINs; qvals = qlevels - 1;
    qminval = parVT.quantMinval;
    chromaarea = qlevels(1)*qlevels(2);
    histSize = chromaarea + qlevels(3);


    [hei,wid,trash]  = size(paddedImage);
    area = hei*wid; % it varies from image to image

    A = rgb2hsv(paddedImage);
    H = round(A(:,:,1) * qvals(1)); % quantized hue
    S = round(A(:,:,2) * qvals(2)); % quantized saturation
    V = round(histeq(A(:,:,3)) * qvals(3)); % quantized brightness
    T = reshape(H + (S * qlevels(1)),[area 1]); % tint = combined hue + sat

    % remove hue+sat from black pixels
    blackspots = (V < qminval);

    hist = zeros(nParts * histSize,1);

    for pp = 1:nParts

        % mask out black spots
        maskVT = resizedMaskSet{pp};
        maskVT(blackspots) = false;
        %maskB = pmskset{ii,pp} & blackspots;
        maskB = resizedMaskSet{pp} & blackspots;
        blacks = sum(maskB(:)) * scaleB;

        grays = accumarray([V(:)+1; qlevels(3)],[maskVT(:); 0]) * scaleV;
        colors = accumarray([T(:)+1; chromaarea],[maskVT(:); 0]);

        indices = (pp-1)*histSize+1:pp*histSize;
        hist(indices) = [blacks; grays(2:end); colors] * (128*64/area); % * weights(pp)
        % dividing by area to normalize the histograms
        % multiplying by 128*64 for historical reasons (didn't divide by area before)
    end
    BVTfeature = hist;

end
