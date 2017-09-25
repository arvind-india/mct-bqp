function p = poly2ccw(p)
	% Return the polygon in counter clock wise order
    % Inputs 
    %  p: input vertices

    % Output 
    %  p: output vertices but sorted

	dist = pdist2(p,p);
	N = size(p,1);
	result = NaN(1,N);
	result(1) = 1;
	for ii=2:N
	    dist(:,result(ii-1)) = Inf;
	    [~, closest_idx] = min(dist(result(ii-1),:));
	    result(ii) = closest_idx;
	end
	p=p(result,:);
	p=p(:,1:2);