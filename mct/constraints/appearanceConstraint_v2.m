function [c_a, weights, Z, y] = appearanceConstraint_v2(k,candidates,dets,image,type,lambda)

  feature_vector_size = 3 * 256; % 3 * Histogram size
  I = eye(feature_vector_size, feature_vector_size);
  weights = zeros(feature_vector_size, candidates);
  c_a = zeros(candidates,k);

  for j=1:candidates
    Z = zeros(k,feature_vector_size);

    %Create a gaussian distribution in that grid. We assume correlations are null
    % TODO pass this as a paremeter
    sigma = sqrt(0.6);
    cov_matrix = [sigma^2 0.0; 0.0 sigma^2];
    gaussian = gauss2d([sqrt(k) sqrt(k)], cov_matrix, round([sqrt(k)/2 sqrt(k)/2]));

    startx = dets(j,3);
    starty = dets(j,4);
    bb_width = dets(j,5);
    bb_height = dets(j,6);
    xstep = bb_width/10;
    ystep = bb_height/10;
    % Create labels
    y = zeros(k,1);
    % Variable to solve sliding bounding boxes off the image
    d = zeros(k,1);
    counter = 1;

    for gridx=-2:2
      for gridy=-2:2
        cx = startx + gridx * xstep;
        cy = starty + gridy * ystep;
        if cy+bb_height <= 800
          y(counter) = gaussian(gridx+3,gridy+3);
          bbimg = imcrop(image,[cx cy bb_width bb_height]);
          R = imhist(bbimg(:,:,1));
          G = imhist(bbimg(:,:,2));
          B = imhist(bbimg(:,:,3));
          Z(counter,:) = [R' G' B'];  % RGB histogram
          d(counter) = 1;
        else
          d(counter) = 0;
        end
        counter = counter + 1;
      end
    end
    %Compute the weights
    if strcmp(type, 'naive') == 1
        % w = inv(Z' * Z + lambda*I) * (Z' * y);
        w = (Z' * Z + lambda*I) \ (Z' * y);
    end
    % According to Afshin this weight is negative
    for i=1:k
      if d(i) == 1
        c_a(j,i) = - real(Z(i,:)*w);
      else
        c_a(j,i) = 0;
      end
    end
    %Store j's weights
    weights(:,j) = w;
  end
end
