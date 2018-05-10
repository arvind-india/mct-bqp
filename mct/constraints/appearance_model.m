function [Zs, ys] = appearance_model(n,targs,image,a_sigma,dx,dy,g_candidates,cameras)
    Zs = cell(length(cameras),1);
    ys = cell(length(cameras),1);
    feature_vector_size = 3 * 256;
    % TODO sometimes k is not g_candidates^2 (like in the inter-cam case)
    T = g_candidates ^ 2; % Number of training examples for the model of the target (same number for every target)
    for i=1:n
        % TODO Create labels
        y = zeros(T,1);
        % TODO Create a gaussian distribution in that grid. We assume correlations are null
        gaussian = gauss2d([sqrt(T) sqrt(T)], a_sigma, round([sqrt(T)/2 sqrt(T)/2]));
        % Get the target location/bb and slide it around in the current frame t
        startx = targs(i,4); starty = targs(i,5);
        bb_width = targs(i,6); bb_height = targs(i,7);
        xstep = bb_width/dx; ystep = bb_height/dy; % Can be arbitrary granularity
        counter = 1;
        Z = zeros(T,feature_vector_size);
        for gridx = -floor(g_candidates/2):floor(g_candidates/2)
          for gridy = -floor(g_candidates/2):floor(g_candidates/2)
              cx = startx + gridx * xstep;
              cy = starty + gridy * ystep;
              bbimg = imcrop(image,[cx cy bb_width bb_height]);
              R = imhist(bbimg(:,:,1));
              G = imhist(bbimg(:,:,2));
              B = imhist(bbimg(:,:,3));
              Z(counter,:) = [R' G' B'];  % RGB histogram
              y(counter) = gaussian(gridx + floor(g_candidates/2 + 1),gridy + floor(g_candidates/2) + 1);
              counter = counter + 1;
          end
        end
        Zs{i} = Z;
        ys{i} = y;
    end
end
