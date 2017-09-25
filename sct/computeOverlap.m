function pixelplane_overlap = computeOverlap(homoplanes)
	% Given homoplanes, compute their intersections
	% Inputs
	%  homoplanes: List of homoplanes vertices

	% Output
	%  pixelplane_overlap: all intersections (i.e points) of the regions
	X = poly2poly(homoplanes{1}',homoplanes{2}');
	intersect = poly2ccw(X');
	if ~isempty(intersect)
	    kinside_points = {};
	    for k=1:size(homoplanes{1},1)
	        if inpoly(homoplanes{1}(k,:),homoplanes{2})
	            kinside_points{end+1} = [k homoplanes{1}(k,:)];
	        end
	    end
	    minside_points = {};
	    for m=1:size(homoplanes{2},1)
	        if inpoly(homoplanes{2}(m,:),homoplanes{1})
	            minside_points{end+1} = [m homoplanes{2}(m,:)];
	        end
	    end
	    kmat = cell2mat(kinside_points);
	    mmat = cell2mat(minside_points);

	    if ~isempty(mmat) && ~isempty(kmat)
	        region = vertcat(kmat(:,2:3),mmat(:,2:3),intersect(:,1:2));
	    elseif isempty(mmat)
	        region = vertcat(kmat(:,2:3),intersect(:,1:2));
	    elseif isempty(kmat)
	        region = vertcat(mmat(:,2:3),intersect(:,1:2));
	    else
	        region = intersect(:,1:2);
	    end

	    new_region = [region ; region(1,:)];
	    pixelplane_overlap = poly2ccw(new_region);
	else
		pixelplane_overlap = 0;
	end