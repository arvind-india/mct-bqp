function [a, weights, c_a, c_ca] = appearance(k, targs_percam, cands_percam, cands_clones, current_frame, next_frame, sigma, prefilter_sigma, lambda, weights, num_cams, filter, dx, dy, dataset)
    use_GPU = 0;
    a = cell(num_cams,1);
    for c = 1:num_cams
        c_a = cell(size(targs_percam{c},1),1);
        c_ca = cell(size(targs_percam{c},1),1);
        for i = 1:size(targs_percam{c},1)
            % fprintf(['\t \t (KCF) Processing "normal" targets for temporal association . Camera: ' num2str(c) ' Target: ' num2str(i) '\n']);
            % Get target
            if strcmp(dataset, 'hda')
                target = targs_percam{c}(i,4:7);
            elseif strcmp(dataset, 'ucla')
                target = targs_percam{c}(i,3:6);
            end
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
            crop_rect(crop_rect < 0) = 0;

            % Gaussian pre-filter
            cornerx = cx - crop_rect(1);
            cornery = cy - crop_rect(2);
            Iblur = imgaussfilt(x, prefilter_sigma);
            ceildx = cast(cornerx, 'int32') + 1;
            ceilbx = cast(cornerx, 'int32') + size(original_bb,2);
            ceildy = cast(cornery, 'int32') + 1;
            ceilby = cast(cornery, 'int32') + size(original_bb,1);
            Iblur((ceildy:ceilby),(ceildx:ceilbx),:) = original_bb(:,:,:);
            x = Iblur;

            % Use features other than RGB
            % TODO original_bb -> HOG(original_bb)
            % TODO x -> HOG(x)
            % [featureVector,hogVisualization] = extractHOGFeatures(x);

            % Generate gaussian labels y
            gsize = [size(x,1) size(x,2)];
            center = [cornerx cornery];
            Sig = [sigma 0; 0 sigma]; % Circular covariance
            y = gauss2d(gsize, Sig , center);

            % Train
            alphaf = train(x, y, sigma, lambda, use_GPU);

            % Get candidate weights
            z = imcrop(next_frame{c}, patch_coords);
            % Use features other than RGB
            % TODO original_bb -> HOG(original_bb)
            responses = detect_patch(alphaf, x, z, sigma, use_GPU);

            xstep = bb_width/dx;
            ystep = bb_height/dy;
            candidate_responses = zeros(k,1);
            t = 1;
            for gridx=-floor(sqrt(k)/2):floor(sqrt(k)/2)
                for gridy=-floor(sqrt(k)/2):floor(sqrt(k)/2)
                    k_cx = cx + gridx * xstep + 1;
                    k_cy = cy + gridy * ystep + 1;

                    % Convert candidate locations in the image to patch locations
                    resp_cx = k_cx - crop_rect(1);
                    resp_cy = k_cy - crop_rect(2);
                    if k_cx > 0 && k_cy > 0 && k_cx < 1024 && k_cy < 576
                        % candidate_responses(gridy + floor(sqrt(k)/2) + 1, gridx + floor(sqrt(k)/2) + 1) = responses(cast(resp_cy, 'int32'), cast(resp_cx, 'int32'));
                        candidate_responses(t) = responses(cast(resp_cy, 'int32'), cast(resp_cx, 'int32'));
                    end
                    t = t + 1;
                end
            end
            c_a{i} = - candidate_responses;


            % fprintf(['\t \t (KCF) Processing "clone" targets for spatial association  Camera: ' num2str(c) ' Target: ' num2str(i) '\n']);
            % Get a patch for this candidate in the frame of the other image (prev)
            candidates_clones = cands_clones{c}{i}(:,1:4);
            clone_responses = ones(k, 1) * 9999999;
            spatial_method = 'bhattacharyya'; % must be kcf or bhattacharyya
            for j=1:k
                if candidates_clones(j,1) ~= 0
                    ccx = candidates_clones(j,1);
                    ccy = candidates_clones(j,2);
                    cbb_width = candidates_clones(j,3);
                    cbb_height = candidates_clones(j,4);
                    if strcmp(spatial_method, 'kcf')
                        z = imcrop(current_frame{getOtherCamIdx(c)}, [ccx - cbb_width ccy - cbb_height 3 * cbb_width 3 * cbb_height]);
                        z_resized = imresize(z, [size(x,1) size(x,2)]);
                        % Resize the patch to be of the same size as the patch that this was trained on?
                        % TODO Alternative is to resize the actual weights OR to train a filter on the candidates and resize to the original one
                        spatial_responses = detect_patch(alphaf, x, z_resized, sigma);
                        spatial_responses = imresize(spatial_responses, [size(z,1) size(z,2)]);
                        clone_responses(j) = avg(spatial_responses(:));
                    elseif strcmp(spatial_method, 'bhattacharyya')
                        % Get the rgb histograms of the original BB
                        h_x = {histcounts(original_bb(:,:,1), 256, 'Normalization','probability') histcounts(original_bb(:,:,2), 256, 'Normalization','probability') histcounts(original_bb(:,:,3), 256, 'Normalization','probability')};
                        % Get the rgb histograms of the clone BB
                        %I1 = [imhist(bb_img1(:,:,1)) imhist(bb_img1(:,:,2)) imhist(bb_img1(:,:,3))];
                        %I2 = [imhist(bb_img2(:,:,1)) imhist(bb_img2(:,:,2)) imhist(bb_img2(:,:,3))];
                        bb_z = imcrop(current_frame{getOtherCamIdx(c)}, [ccx ccy cbb_width cbb_height]);
                        h_z = {histcounts(bb_z(:,:,1), 256, 'Normalization','probability') histcounts(bb_z(:,:,2), 256, 'Normalization','probability') histcounts(bb_z(:,:,3), 256, 'Normalization','probability')};
                        %distance_metric = 'euclidean'; % emd, cosine, euclidean, chisq, L1
                        %appearance_score = pdist2(I1',I2',distance_metric);
                        % Get the sum of the bhattacharyya distances between histograms
                        clone_responses(j) = 1 / bhattacharyya(h_x, h_z);
                    end
                end
            end
            c_ca{i} = clone_responses;

        end
        a{c} = [cell2mat(c_a) ; cell2mat(c_ca)];
        % TODO Recursive filter (use weights)
        if strcmp(filter, 'recursive')
            a{c} = 0.5 * a{c} + 0.5 * weights{c};
        end

    end
    weights = a;
end

function oi = getOtherCamIdx(idx)
  oi = rem(idx,2) + 1; % TODO This is a dirty hack to get 1 if 2 or 2 if 1
end


function bdist = bhattacharyya(h1, h2)
    % N.B. both histograms must be normalised
    % (i.e. bin values lie in range 0->1 and SUM(bins(i)) = 1
    %       for i = {histogram1, histogram2} )
    % where each histogram is a 1xN vector
    % get number of bins
    % (i.e. dimension 2 from Nx1 inputs)
    bdist = 0;
    for j = 1:3
        histogram1 = h1{j};
        %histogram1(1) = 0;
        histogram2 = h2{j};
        %histogram2(1) = 0;
        bins = size(histogram1, 2);

        % estimate the bhattacharyya co-efficient

        bcoeff = 0;
        for i=1:bins

            bcoeff = bcoeff + sqrt(histogram1(i) * histogram2(i));

        end

        % get the distance between the two distributions as follows

        bdist = bdist + sqrt(1 - bcoeff);
    end
end

function im = prefilter(x, original_bb, cdx, cdy, sigma)
    Iblur = imgaussfilt(x, sigma);
    ceildx = cast(cdx, 'int32') + 1;
    ceilbx = cast(cdx, 'int32') + size(original_bb,2);
    ceildy = cast(cdy, 'int32') + 1;
    ceilby = cast(cdy, 'int32') + size(original_bb,1);
    Iblur((ceildy:ceilby),(ceildx:ceilbx),:) = original_bb(:,:,:);
    im = Iblur;
end

function k = kernel_correlation(x1, x2, sigma)
    c = ifft2(sum(conj(fft2(x1)) .* fft2(x2), 3));
    a = x1(:);
    b = x2(:);
    d = (a')*a + (b')*b - 2*c;
    k = exp(-1 / sigma^2 * abs(d) / numel(d));
end

function alphaf = train(x, y, sigma, lambda, use_GPU)
    x = cast(x, 'double');
    if use_GPU
        x = gpuArray(x);
    end
    k = kernel_correlation(x, x, sigma);
    a = fft2(y) ./ (fft2(k) + lambda);
    if use_GPU
        alphaf = gather(a);
    else
        alphaf = a;
    end
end

function responses = detect_patch(alphaf, x, z, sigma, use_GPU)
    x = cast(x, 'double');
    z = cast(z, 'double');
    if use_GPU
        x = gpuArray(x);
        z = gpuArray(z);
    end
    k = kernel_correlation(z, x, sigma);
    r = real(ifft2(alphaf .* fft2(k)));
    if use_GPU
        responses = gather(r);
    else
        responses = r;
    end
end


%Image descriptor based on Histogram of Orientated Gradients for gray-level images. This code
%was developed for the work: O. Ludwig, D. Delgado, V. Goncalves, and U. Nunes, 'Trainable
%Classifier-Fusion Schemes: An Application To Pedestrian Detection,' In: 12th International IEEE
%Conference On Intelligent Transportation Systems, 2009, St. Louis, 2009. V. 1. P. 432-437. In
%case of publication with this code, please cite the paper above.

function hmat = HOG(Im,nwin_x,nwin_y, B)
    % nwin_x=77;%set here the number of HOG windows per bound box
    % nwin_y=45;
    % B=31;%set here the number of histogram bins
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
    hmat = zeros(nwin_y,nwin_x,B);
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
            hmat(n+1,m+1,:) = H2;
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
