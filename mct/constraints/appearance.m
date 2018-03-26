function [c_a, w, Z, y] = appearance(k,n,targs,cands,image,next_image,type,lambda,a_sigma,dx,dy,g_candidates,weights)

  feature_vector_size = 3 * 256; % 3 * Histogram size = Feature vector
  I = eye(feature_vector_size, feature_vector_size);
  w = zeros(feature_vector_size, n);
  c_a = zeros(k,n);

  T = k; % Number of training examples for the model of the target (same number for every target)

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

    % Get feature vectors for candidates in the next frame t+1
    Phi = zeros(k,feature_vector_size);
    for j=1:k
        bbimg = imcrop(next_image,cands{i}(j,1:4)); % One candidate BB
        R = imhist(bbimg(:,:,1));
        G = imhist(bbimg(:,:,2));
        B = imhist(bbimg(:,:,3));
        Phi(j,:) = [R' G' B'];  % RGB histogram
    end
    for m = 1:feature_vector_size
        maxim = max(Z(:,m));
        if maxim ~= 0
            Z(:,m) = Z(:,m)./maxim;
            Phi(:,m) = Phi(:,m)./maxim;
        end
    end
    %Compute the weights
    if strcmp(type, 'naive') == 1
        % w = inv(Z' * Z + lambda*I) * (Z' * y);
        w_i = (Z' * Z + lambda*I) \ (Z' * y);
    end
    % According to Afshin this weight is negative
    for j=1:k
      if y(j) == 0
          c_a(j,i) = 0;
      else
          if ~isempty(weights)
              % TODO integrate new and old weights
              %c_a(j,i) = -Phi(j,:) * weights(:,i);
              c_a(j,i) = -Phi(j,:) * w_i;
          else
              c_a(j,i) = -Phi(j,:) * w_i;
          end
      end
    end
    %Store i's weights
    w(:,i) = w_i;
  end
  % Convert c_a from a n by k matrix to a n*k vector as it should be
  c_a = c_a(:);
end
