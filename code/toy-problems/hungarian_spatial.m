% Fictional asymetric assignment with 2 cameras, one with 3 people and the other with 2 people

% We assume they are all in the overlap regions for now...
% Camera plane regions
%cam1_region =
%cam2_region =
%overlap =
tic
H1 = [0.043383 0.043388 35.129;
     -0.035325 0.022121	68.004;
     0.0020342 0.019584	1];
H2 = [0.053286 0.10144 -37.736;
      0.017139 0.0081285 45.967;
      0.0016208	0.016526 1];

% Load images of the same frame in the two images
cam1_img = imread('1147.png');
cam2_img = imread('1017.png');
% List of peds in the two cameras
% cam frame x y width height bit ped
% 2 peds - camera 57 -- allDetections{1}{1147}
cam_1_peds = [57 1147 108 161 262 591 1 15 ; ...
57 1147 559 159 271 637 1 52];
%cam_1_peds = [57 1147 280 90 130 390 1 52; ...
%57	1147 50 90 130 290 1 15];
% 3 peds - camera 58 -- allDetections{2}{1017}
% The third pedestrian, ped52 actually wasn't a valid detection since it's feet are not visible, but lets put him in for the sake of the experiment
cam_2_peds = [58 1017 3 250 325 542 1 23; 58 1017 575 144 198 608 1 15; ...
 58 1017 275 295 336 501 1 52];

% Convert positions to the ground plane and draw Histograms simultaneously
cam_1_peds_gnd = drawHistograms_computeGndDetections(cam_1_peds, cam1_img, H1);
cam_2_peds_gnd = drawHistograms_computeGndDetections(cam_2_peds, cam2_img, H2);
figure
%openfig('~/mct-bqp/hda_data/homographies/7th_floor_ground_plane_reference_frame_map.fig');
hold on;
for i=1:size(cam_1_peds_gnd,1)
  if cam_1_peds(i,8) == 15
    scatter(cam_1_peds_gnd(i,1),cam_1_peds_gnd(i,2),'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red'));
  elseif cam_1_peds(i,8) == 52
    scatter(cam_1_peds_gnd(i,1),cam_1_peds_gnd(i,2),'MarkerFaceColor',rgb('Cyan'),'MarkerEdgeColor',rgb('Cyan'));
  elseif cam_1_peds(i,8) == 23
    scatter(cam_1_peds_gnd(i,1),cam_1_peds_gnd(i,2),'MarkerFaceColor',rgb('Green'),'MarkerEdgeColor',rgb('Green'));
  end
end

for i=1:size(cam_2_peds_gnd,1)
  if cam_2_peds(i,8) == 15
    scatter(cam_2_peds_gnd(i,1),cam_2_peds_gnd(i,2),'MarkerFaceColor',rgb('Red'),'MarkerEdgeColor',rgb('Red'));
  elseif cam_2_peds(i,8) == 52
    scatter(cam_2_peds_gnd(i,1),cam_2_peds_gnd(i,2),'MarkerFaceColor',rgb('Cyan'),'MarkerEdgeColor',rgb('Cyan'));
  elseif cam_2_peds(i,8) == 23
    scatter(cam_2_peds_gnd(i,1),cam_2_peds_gnd(i,2),'MarkerFaceColor',rgb('Green'),'MarkerEdgeColor',rgb('Green'));
  end
end

% Solve assignment (construct score matrix)
S = zeros(2,3); P = zeros(2,3); A = zeros(2,3); V = zeros(2,3);
assignmentAlgo = 'jonker_volgenant';
for a=1:size(S,1)
    for b=1:size(S,2)
        % Distance
        p1 = cam_1_peds_gnd(a,:);
        p2 = cam_2_peds_gnd(b,:);
        P(a,b) = sqrt((p1(1)-p2(1))^2 + (p1(2)-p2(2))^2);
        % Velocity
        v1 = 0;
        v2 = 0;
        V(a,b) = sum(abs(v1 - v2));
        % Appearance
        bb_img1 = imcrop(cam1_img,cam_1_peds(a,3:6));
        bb_img2 = imcrop(cam2_img,cam_2_peds(b,3:6));
        %imshow(bb_img1);
        %waitforbuttonpress
        %imshow(cam2_img);
        %imshow(bb_img2);
        %waitforbuttonpress
        I1 = [imhist(bb_img1(:,:,1)) imhist(bb_img1(:,:,2)) imhist(bb_img1(:,:,3))];
        I2 = [imhist(bb_img2(:,:,1)) imhist(bb_img2(:,:,2)) imhist(bb_img2(:,:,3))];
        % Distances
        %'euclidean' / 'sqeuclidean':
        %  Euclidean / SQUARED Euclidean distance.  Note that 'sqeuclidean'
        %  is significantly faster.
        %'chisq'
        %  The chi-squared distance between two vectors is defined as:
        %   d(x,y) = sum( (xi-yi)^2 / (xi+yi) ) / 2;
        %  The chi-squared distance is useful when comparing histograms.
        %'cosine'
        %  Distance is defined as the cosine of the angle between two vectors.
        %'emd'
        %  Earth Mover's Distance (EMD) between positive vectors (histograms).
        %  Note for 1D, with all histograms having equal weight, there is a simple
        %  closed form for the calculation of the EMD.  The EMD between histograms
        %  x and y is given by the sum(abs(cdf(x)-cdf(y))), where cdf is the
        %  cumulative distribution function (computed simply by cumsum).
        %'L1'
        %  The L1 distance between two vectors is defined as:  sum(abs(x-y));
        distance_metric = 'euclidean'; % emd, cosine, euclidean, chisq, L1
        appearance_score = pdist2(I1',I2',distance_metric);
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
figure
bar(A(:));

% Normalize V along the smaller dimension
for a=1:size(V,1)
  v_sum = sum(V(a,:));
  for b=1:size(V,1)
    V(a,b) = V(a,b)/v_sum;
  end
end
% Calculate S
option = 'multiplication'
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
    assignments = lapjv(S,resolution);
time = toc;
fprintf(['Time: ', num2str(round(time*100)/100), ' seconds \n']);
fprintf('Optimal assignment found: \n');
for i=1:size(S,1)
  fprintf('Best assignment for ped %d in cam %d === ped %d in cam %d \n', cam_1_peds(i,8), cam_1_peds(i,1),...
  cam_2_peds(assignments(i),8), cam_2_peds(assignments(i),1));
end

function t = H(matrix, point)
    t = matrix * [point; 1];
    t(1) = t(1)/t(3);
    t(2) = t(2)/t(3);
    t = t(1:2);
end

function cam_peds_gnd = drawHistograms_computeGndDetections(peds_data, cam_img, h_matrix)
  n = size(peds_data, 1);
  cam_peds_gnd = zeros(n,2);
  rgb_histograms = {};
  figure
  hold on
  for i=1:n
    cam_peds_gnd(i,:) = H(h_matrix,[peds_data(i,3) + peds_data(i,5)/2; peds_data(i,4) + peds_data(i,6)/2]);
    bb_img = imcrop(cam_img,peds_data(i,3:6));
    I = [imhist(bb_img(:,:,1)) imhist(bb_img(:,:,2)) imhist(bb_img(:,:,3))];
    rgb_histograms{i} = I;
    for j=1:3
      subplot(n,3,j+3*(i-1))
      if j == 1
        bar(rgb_histograms{i}(:,j), 'r')
        title(['Ped:', num2str(peds_data(i,8)), ' R']);
      elseif j == 2
        bar(rgb_histograms{i}(:,j), 'g')
        title(['Ped:', num2str(peds_data(i,8)), ' G']);
      elseif j == 3
        bar(rgb_histograms{i}(:,j), 'b')
        title(['Ped:', num2str(peds_data(i,8)), ' B']);
      end
    end
  end
end

function [S, A, P, V] = createScoreMatrix(f,n_ov1,n_ov2,targs_overlap,images,d1_metric,d256_metric, motion_models_overlap)
    S = zeros(n_ov1,n_ov2); A = zeros(n_ov1,n_ov2);
    P = zeros(n_ov1,n_ov2); V = zeros(n_ov1,n_ov2);

    for a=1:n_ov1
        for b=1:n_ov2

            % TODO Euclidean Distance
            p1 = targs_overlap{1}(a,8:9);
            p2 = targs_overlap{2}(b,8:9);
            P(a,b) = pdist([p1;p2],d1_metric);

            % TODO Euclidean distance between motion cues
            if f == 1
                v1 = 0;
                v2 = 0;
            else
                v1 = motion_models_overlap{1}(a,4:5);
                v2 = motion_models_overlap{2}(b,4:5);
            end
            V(a,b) = sum(abs(v1 - v2))/sum(abs(v1 + v2));

            % TODO Appearance
            bb_img1 = imcrop(images{1},targs_overlap{1}(a,4:7));
            bb_img2 = imcrop(images{2},targs_overlap{2}(b,4:7));
            %imshow(bb_img1);
            %waitforbuttonpress
            %imshow(cam2_img);
            %imshow(bb_img2);
            %waitforbuttonpress
            I1 = [imhist(bb_img1(:,:,1)) imhist(bb_img1(:,:,2)) imhist(bb_img1(:,:,3))];
            I2 = [imhist(bb_img2(:,:,1)) imhist(bb_img2(:,:,2)) imhist(bb_img2(:,:,3))];
            % Chi-squared between hists
            appearance_score = pdist2(I1',I2',d256_metric);
            A(a,b) = (1/3)*sum(appearance_score(:,1),1) / (256*max(appearance_score(:,1))) + (1/3)*sum(appearance_score(:,2),1)/(256*max(appearance_score(:,2))) + (1/3)* sum(appearance_score(:,3),1)/(256*max(appearance_score(:,3)));

            %TODO Ground plane mapped bounding box overlap ? ofc this performs better as homographies improve
            % Is this a good approach? Ground plane is a birds eye view so this mapping might not be correct
            % TODO Map 4 corners of target a to the ground plane
            % TODO Map 4 corners of target b to the ground plane
            % Compute area of intersection
            % Compute area of intersection

            %iou = area_intersection/area_union;
        end
    end
    %S = A + V + P;
    if f == 1
        S = (A/max(A(:))) .* (P/max(P(:)));
    else
        S = (A/max(A(:))) .* (V/max(V(:))) .* (P/max(P(:)));
    end
end

function valid_matchings = getValidMatchings_lapjv(i, S, score_threshold, n_ov2, n_ov1, targs_in_overlap, assignments, valid_matchings)
    if n_ov2 >= n_ov1
        fprintf('\t\tBest assignment for id %d in cam %d === id %d in cam %d \n', i, targs_in_overlap{1}(i,1),...
        assignments(i), targs_in_overlap{2}(assignments(i),1));
        score = S(i,assignments(i));
    else
        fprintf('\t\tBest assignment for id %d in cam %d === id %d in cam %d \n', assignments(i), targs_in_overlap{1}(assignments(i),1),...
        i, targs_in_overlap{2}(i,1));
        score = S(assignments(i),i);
    end
    fprintf('\t\tScore: %f\n', score);
    % TODO If its a good score, then store merge
    if score < score_threshold
        % TODO Make this work for more than 2 cameras
        if n_ov2 >= n_ov1
            valid_matchings{1}{end+1} = targs_in_overlap{1}(i,:);
            valid_matchings{2}{end+1} = targs_in_overlap{2}(assignments(i),:);
        else
            valid_matchings{1}{end+1} = targs_in_overlap{1}(assignments(i),:);
            valid_matchings{2}{end+1} = targs_in_overlap{2}(i,:);
        end
    end
end
