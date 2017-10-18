function drawPoly(p, color, width, f)
	% Draw a polygon
	% Inputs
	%  p: sequence of points, ordered clock or counter-clock wise
	%  color: color of the polygon
	%  eidth: LineWidth for the polygon drawn
	%  f: option to fill the polygon

	% Output
	%  homoplanes: ground planes for each camera

	x = [p(:,1); p(1,1)];
	y = [p(:,2); p(1,2)];
	colorvec = rgb(color);
	plot(x,y, 'Color', colorvec, 'LineWidth', width);
	if f == true
		fill(p(:,1),p(:,2), colorvec+([1.0 1.0 1.0]-colorvec)./2)
	end
