function [c_a, allbbs, allbb_imgs] = appearanceConstraint(n,k,f,allDetections,cameraListImages,lambda,type,dataset,id)
    %%==============================================================================
    if strcmp(dataset,'campus_2') == 1
        feature_vector_size = 3;
        I = eye(feature_vector_size,feature_vector_size);
        weights = zeros(feature_vector_size,n);
        allbbs = cell(n,1);
        allbb_imgs = cell(n,1);
        c_a = cell(n,1);
        for j=1:n
            Z = zeros(k,feature_vector_size);
            %Get the bounding box j
            bb = allDetections{1}{f}(j,:);
            bbwidth = bb(5);
            bbheight = bb(6);

            %Get candidate bounding boxes and compute y
            bbimg = cell(k,1);
            bbpos = cell(k,1);

            %Create a gaussian distribution in that grid
            gaussian = gauss2d([sqrt(k) sqrt(k)], [0.6 0.6], [3 3]);
            %Go over the grid and atribute values of y and get the bounding boxes
            xstep = bbwidth/sqrt(k);
            ystep = bbheight/sqrt(k);

            startx = (bb(3)+bbwidth/2)-2*xstep;
            starty = (bb(4)+bbheight/2)-2*ystep;

            % Create labels
            y = zeros(k,1);
            for gridx=1:sqrt(k)
                for gridy=1:sqrt(k)
                    cx = startx + (gridx-1)*xstep;
                    cy = starty + (gridy-1)*ystep;
                    bbx = cx-bbwidth/2;
                    bby = cy-bbheight/2;
                    y(sqrt(k)*(gridy-1)+gridx) = gaussian(gridx,gridy);
                    % TODO change this, should work end+1, to sqrt(k)*grix+gridy
                    bbimg{sqrt(k)*gridx+gridy} = imcrop(imread(cameraListImages{1}{f}),[bbx bby bbwidth bbheight]);
                    bbpos{sqrt(k)*gridx+gridy} = [bbx bby bbwidth bbheight];
                end
            end

            %Compute features
            for i=1:k
                feat = zeros(feature_vector_size,1);
                feat(1) = mean2(bbimg{i}(:,:,1)); %R
                feat(2) = mean2(bbimg{i}(:,:,2)); %G
                feat(3) = mean2(bbimg{i}(:,:,3)); %B
                Z(i,:) = feat;
            end
            %Compute the weights
            %TODO: This can be done much more efficiently in the Fourier Domain as proposed in High Speed Kernelized Correlation Filters
            %Zfourier = fft(Z);
            %yfourier = fft(y);
            %w = inv(Z' * Z + lambda*I) * (Z' * y);
            if strcmp(type, 'naive') == 1
                w = (Z' * Z + lambda*I) \ (Z' * y);
            end
            c_a{j} = zeros(k,1);
            for i=1:k
                c_a{j}(i) = -real(Z(i,:)*w);
            end
            %Store j's weights
            weights(:,j) = w;
            allbbs{j} = cell2mat(bbpos);
            allbb_imgs{j} = bbimg;
        end
        c_a = cell2mat(c_a);
    end

    %%==============================================================================
    if strcmp(dataset,'hda') == 1
        feature_vector_size = 3 * 256; % 3 * Histogram size
        I = eye(feature_vector_size, feature_vector_size);
        weights = zeros(feature_vector_size,n);
        allbbs = cell(n,1);
        allbb_imgs = cell(n,1);
        c_a = cell(n,1);
        for j=1:n
            Z = zeros(k,feature_vector_size);

            %Get the bounding box j
            bb = allDetections{id}{f}(j,:);
            bbwidth = bb(5);
            bbheight = bb(6);

            %Get candidate bounding boxes and compute y
            bbimg = cell(k,1);
            bbpos = cell(k,1);

            %Create a gaussian distribution in that grid. We assume correlations are null
            % TODO pass this as a paremeter
            sigma = sqrt(0.6);
            cov_matrix = [sigma^2 0.0; 0.0 sigma^2];
            if mod(sqrt(k),2) == 0
                disp('Please make sqrt(k) odd.')
            end
            gaussian = gauss2d([sqrt(k) sqrt(k)], cov_matrix, round([sqrt(k)/2 sqrt(k)/2]));
            %Go over the grid and atribute values of y and get the bounding boxes
            xstep = bbwidth/sqrt(k);
            ystep = bbheight/sqrt(k);

            startx = (bb(3) + bbwidth/2)-2*xstep;
            starty = (bb(4) + bbheight/2)-2*ystep;

            % Create labels
            % TODO: Try to vectorize this for speed (at least y)
            y = zeros(k,1);
            for gridx = 1:sqrt(k)
                for gridy = 1:sqrt(k)
                    cx = startx + (gridx-1) * xstep;
                    cy = starty + (gridy-1) * ystep;
                    % TODO change this, should work end+1, to sqrt(k)*grix+gridy
                    y(sqrt(k) * (gridy-1) + gridx) = gaussian(gridx, gridy);

                    bbx = cx-bbwidth/2;
                    bby = cy-bbheight/2;
                    bbimg{sqrt(k) * (gridx-1)+gridy} = imcrop(cameraListImages{id}{f},[bbx bby bbwidth bbheight]);
                    bbpos{sqrt(k) * (gridx-1)+gridy} = [bbx bby bbwidth bbheight];
                end
            end

            %Compute features
            for i=1:k
                R = imhist(bbimg{i}(:,:,1));
                G = imhist(bbimg{i}(:,:,2));
                B = imhist(bbimg{i}(:,:,3));
                Z(i,:) = [R' G' B'];  % RGB histogram
            end
            %Compute the weights
            if strcmp(type, 'naive') == 1
                % w = inv(Z' * Z + lambda*I) * (Z' * y);
                w = (Z' * Z + lambda*I) \ (Z' * y);
            end
            if strcmp(type, 'hkf') == 1
                %TODO: This can be done much more efficiently in the Fourier Domain,
                % Zfourier=fft(Z), yfourier=fft(y), as proposed in High Speed Kernelized Correlation Filters
            end
            c_a{j} = zeros(k,1);
            % According to Afshin this weight is negative
            for i=1:k
                c_a{j}(i) = - real(Z(i,:)*w);
            end
            %Store j's weights
            weights(:,j) = w;
            allbbs{j} = cell2mat(bbpos);
            allbb_imgs{j} = bbimg;
        end
        c_a = cell2mat(c_a);
    end
