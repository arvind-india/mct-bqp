function [c_a, allbbs] = appearanceConstraint(n,k,f,allDetections,cameraListImages,lambda,type)

    I = eye(3,3);
    weights = zeros(3,n);
    allbbs = {};

    c_a = cell(n,1);
    for j=1:n
        Z = zeros(k,3);
        %Get the bounding box j
        bb = allDetections{1}{f}(j,:);
        bbwidth = bb(5);
        bbheight = bb(6);

        bbimg = cell(k,1);
        bbpos = cell(k,1);
        %Get candidate bounding boxes and compute y
        bbimg = {};

        %Create a gaussian distribution in that grid
        gaussian = gauss2d([5 5], [0.6 0.6], [3 3]);
        %Go over the grid and atribute values of y and get the bounding boxes
        xstep = bbwidth/5;
        ystep = bbheight/5;

        startx = (bb(3)+bbwidth/2)-2*xstep;
        starty = (bb(4)+bbheight/2)-2*ystep;

        y = zeros(k,1);
        for gridx=1:5
            for gridy=1:5
                cx = startx + (gridx-1)*xstep;
                cy = starty + (gridy-1)*ystep;
                bbx = cx-bbwidth/2;
                bby = cy-bbheight/2;
                y(sqrt(k)*(gridy-1)+gridx) = gaussian(gridx,gridy);
                bbimg{end+1} = imcrop(imread(cameraListImages{1}{f}),[bbx bby bbwidth bbheight]);
                bbpos{end+1} = [bbx bby bbwidth bbheight];
            end
        end

        %Compute features
        for i=1:k
            feat = zeros(3,1);
            feat(1) = mean2(bbimg{i}(:,:,1)); %R
            feat(2) = mean2(bbimg{i}(:,:,2)); %G
            feat(3) = mean2(bbimg{i}(:,:,3)); %B
            Z(i,:) = feat;
        end
        %Compute the weights
        %TODO: This can be done much more efficiently in the Fourier Domain as proposed in High Speed Kernelized Correlation Filters
        %Zfourier = fft(Z);
        %yfourier = fft(y);
        w = inv(Z' * Z + lambda*I) * (Z' * y);

        c_a{j} = zeros(k,1);
        for i=1:k
            c_a{j}(i) = -real(Z(i,:)*w);
        end
        %Store j's weights
        weights(:,j) = w;
        allbbs{j} = cell2mat(bbpos);
    end
    c_a = cell2mat(c_a);
