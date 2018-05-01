% Fictional asymetric assignment with 2 cameras, one with 3 people and the other with 2 people
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
cam_1_peds = [57 1147 108 161 262 591 1 15 ; 57 1147 559 159 271 637 1 52];
% 3 peds - camera 58 -- allDetections{2}{1017}
% The third pedestrian, ped52 actually wasn't a valid detection since it's feet are not visible, but lets put him in for the sake of the experiment
cam_2_peds = [58 1017 3 250 325 542 1 23; 58 1017 575 144 198 608 1 15; 58 1017 275 295 336 501 1 52];

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

% Solve spatial assignment using Frank-Wolfe


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
