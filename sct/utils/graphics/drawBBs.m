function drawBBs(dets, bb_color)

    bb_offset = 12;
    bb_linewidth = 1;
    for k = 1:size(dets,1)
        score_str = sprintf('%0.2f',dets(k,5));
        H = text(double(dets(k,1)), double(dets(k,2))-bb_offset, score_str, 'color', bb_color); %x y
        hold on
        rectangle('Position',dets(k,1:4),'EdgeColor', bb_color, 'LineWidth', bb_linewidth);
    end
