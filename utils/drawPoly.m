function drawPoly(p, color, width, f)

	x = [p(:,1); p(1,1)];
	y = [p(:,2); p(1,2)];
	colorvec = rgb(color);
	plot(x,y, 'Color', colorvec, 'LineWidth', width);
	if f == true
		fill(p(:,1),p(:,2), colorvec+([1.0 1.0 1.0]-colorvec)./2)
	end