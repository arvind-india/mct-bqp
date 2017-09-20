function pixelplane_overlap = computeintersect(homoplanes)

	[xi,yi] = intersections(homoplanes{1}(:,1),homoplanes{1}(:,2),homoplanes{2}(:,1),homoplanes{2}(:,2));
	intersect = horzcat(xi,yi);
	if ~isempty(intersect)
	    scatter(intersect(:,1),intersect(:,2),'MarkerFaceColor',rgb('Black'));
	    edges_i = {};
	    edges_j = {};
	    for li = 1:size(homoplanes{1},1)
	        edges_i{li} = [li mod(li,size(homoplanes{1},1))+1];
	    end
	    for lj = 1:size(homoplanes{2},1)
	        edges_j{lj} = [lj mod(lj,size(homoplanes{2},1))+1];
	    end
	    kinside_points = {};
	    for k=1:size(homoplanes{1},1)
	        if inpoly(homoplanes{1}(k,:),homoplanes{2},cell2mat(edges_j'))
	            kinside_points{end+1} = [k homoplanes{1}(k,:)];
	        end
	    end
	    minside_points = {};
	    for m=1:size(homoplanes{2},1)
	        if inpoly(homoplanes{2}(m,:),homoplanes{1},cell2mat(edges_i'))
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
	    drawPoly(new_region,'Black',1.5,false)
	    pixelplane_overlap = homographyCampusTransform(intersect,invhomographies,0);
	else
		pixelplane_overlap = 0;
	end
