function drawBBs(dets, bb_color, bb_linewidth)
  for k = 1:size(dets,1)
      hold on
      rectangle('Position',dets(k,1:4),'EdgeColor', bb_color, 'LineWidth', bb_linewidth);
  end
end
