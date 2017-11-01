function drawBBs(dets, bb_color)
    bb_linewidth = 1;
    for k = 1:size(dets,1)
        hold on
        rectangle('Position',dets(k,3:6),'EdgeColor', bb_color, 'LineWidth', bb_linewidth);
    end
end
