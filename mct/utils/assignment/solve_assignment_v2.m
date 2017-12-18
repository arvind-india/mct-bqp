function [assignments, S] = solve_assignment_v2(S, images, targs, assignmentAlgorithm)
  % Solve assignment
  P = S; A = S; V = S;
  for a=1:size(S,1)
      for b=1:size(S,2)
          % Distance
          p1 = targs{1}(a,9:10);
          p2 = targs{2}(b,9:10);
          P(a,b) = sqrt((p1(1)-p2(1))^2 + (p1(2)-p2(2))^2);
          % Velocity
          v1 = 0;
          v2 = 0;
          V(a,b) = sum(abs(v1 - v2));
          % Appearance
          bb_img1 = imcrop(images{1},targs{1}(a,3:6));
          bb_img2 = imcrop(images{2},targs{2}(b,3:6));
          %imshow(bb_img1);
          %waitforbuttonpress
          %imshow(cam2_img);
          %imshow(bb_img2);
          %waitforbuttonpress
          I1 = [imhist(bb_img1(:,:,1)) imhist(bb_img1(:,:,2)) imhist(bb_img1(:,:,3))];
          I2 = [imhist(bb_img2(:,:,1)) imhist(bb_img2(:,:,2)) imhist(bb_img2(:,:,3))];
          % Chi-squared between hists
          appearance_score = pdist2(I1',I2','chisq');
          A(a,b) = (1/3)*sum(appearance_score(:,1),1) / (256*max(appearance_score(:,1))) + (1/3)*sum(appearance_score(:,2),1)/(256*max(appearance_score(:,2))) + (1/3)* sum(appearance_score(:,3),1)/(256*max(appearance_score(:,3)));
      end
  end
  % Normalize P along the smaller dimension
  for a=1:size(P,1)
    p_sum = sum(P(a,:));
    for b = 1:size(P,2)
      P(a,b) = P(a,b)/p_sum;
    end
  end

  % Normalize A along the smaller dimension
  for a=1:size(A,1)
    a_sum = sum(A(a,:));
    for b=1:size(A,2)
      A(a,b) = A(a,b)/a_sum;
    end
  end

  % Normalize V along the smaller dimension
  for a=1:size(V,1)
    v_sum = sum(V(a,:));
    for b=1:size(V,1)
      V(a,b) = V(a,b)/v_sum;
    end
  end
  % Calculate S
  option = 'multiplication';
  for a=1:size(S,1)
    for b=1:size(S,2)
      %S(a,b) = A(a,b);
      %S(a,b) = P(a,b); % Even though the actual values are not optimal, the attribution is since it manages to find the optimal sum
      if strcmp('multiplication', option)
        S(a,b) = A(a,b) * P(a,b);
      elseif strcmp('addition', option)
        S(a,b) = A(a,b) + P(a,b);
      end
    end
  end

  resolution = eps; % NOTE This can be changed to accelerate the algorithm
  tic
      if strcmp(assignmentAlgorithm,'jonker_volgenant')
        assignments = lapjv(S,resolution);
      end
  time = toc;
  fprintf(['\t Solving assignment took: ', num2str(round(time*100)/100), '\n']);
end
