function drawBBs(dets, bb_color, dataset)
    % Draw a bounding box given a detection and a colour
    % Inputs
    %  dets: numerical values of the detection
    %  colour: colour in rgb triplets
    if strcmp(dataset, 'campus_2')
      bb_offset = 12;
      bb_linewidth = 1;
      for k = 1:size(dets,1)
          score_str = sprintf('%0.2f',dets(k,5));
          H = text(double(dets(k,1)), double(dets(k,2))-bb_offset, score_str, 'color', bb_color); %x y
          hold on
          rectangle('Position',dets(k,1:4),'EdgeColor', bb_color, 'LineWidth', bb_linewidth);
      end
    elseif strcmp(dataset, 'hda')
      bb_offset = 12;
      bb_linewidth = 1;
      for k = 1:size(dets,1)
          %score_str = sprintf('%0.2f',dets(k,5));
          %H = text(double(dets(k,1)), double(dets(k,2))-bb_offset, score_str, 'color', bb_color); %x y
          hold on
          rectangle('Position',dets(k,1:4),'EdgeColor', bb_color, 'LineWidth', bb_linewidth);
      end
    end
