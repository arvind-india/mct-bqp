function [c_a, w] = appearance(k,n,cands,next_image,type,lambda,weights,filter,Zs,ys)

  feature_vector_size = 3 * 256; % 3 * Histogram size = Feature vector
  I = eye(feature_vector_size, feature_vector_size);
  w = zeros(feature_vector_size, n);
  c_a = zeros(k,n);

  for i=1:n
    % TODO Get models that are passed in
    Z = Zs{i};
    y = ys{i};

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
    % According to Afshin these weights must be negative
    for j=1:k
      if y(j) == 0
          c_a(j,i) = 0;
      else
          if strcmp(filter,'recursive')
              % TODO integrate new and old weights
              c_a(j,i) = - Phi(j,:) * (0.5 * w_i + 0.5 * weights(:,i));
          elseif strcmp(filter,'non-recursive')
              % TODO how to do this one?
              c_a(j,i) = -Phi(j,:) * w_i;
          elseif strcmp(filter,'none')
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
