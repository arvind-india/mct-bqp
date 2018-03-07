% The main goal of this file is to detect outliers by seeing detections that appear constantly at the same place
% Plot (x,y) over time (all frames, z-axis, try to find sequences with small variances in this axis)
function identifyPotentialStaticObjects(allDetections)
  % Plot 3D frames
  figure
  hold on
  dets = allDetections{1};
  for i=1:size(dets,1)
    for j=1:size(dets{i},1)
      line = dets{i}(j,:);
      scatter3(line(3)+line(5)/2,line(4)+line(6),i,17, 'filled');
    end
  end
