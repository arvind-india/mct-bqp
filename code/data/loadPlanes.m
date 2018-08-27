function [inplanes, ground_plane_regions, overlap] = loadPlanes(visibility_regions_directory, homographies, cameras, num_cams, dataset)

    inplanes = cell(num_cams,1);
    if strcmp(dataset, 'hda')
        for i=1:num_cams
            inplanes{i} = load(strcat(visibility_regions_directory, num2str(cameras{i}),'.mat'));
            inplanes{i} = inplanes{i}.t;
        end
        ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, num_cams, dataset);
        [overlap, ~, ~] = computeOverlap(ground_plane_regions);
    elseif strcmp(dataset, 'ucla')
        ground_plane_regions = cell(num_cams,1);
        for i=1:num_cams
            ground_plane_regions{i} = csvread(strcat(visibility_regions_directory, '/region' ,num2str(i), '.txt'));
        end
        [overlap, ~, ~] = computeOverlap(ground_plane_regions);
    end
end

function homoplanes = computeGroundPlaneRegions(inplanes, homographies, num_cameras, dataset)
    % Given a list of points representing regions put them in the ground plane
    % Inputs
    %  homographies: the homographies for each camera
    %  inplanes: representative planes of each camera

    % Output
    %  homoplanes: ground planes for each camera

    homoplanes = homographyTransform(inplanes,homographies,num_cameras,dataset);

    for i=1:num_cameras
        homoplanes{i} = [homoplanes{i};homoplanes{i}(1,:)];
        inplanes{i} = [inplanes{i};inplanes{i}(1,:)];
    end
end


function [pixelplane_overlap, kmat, mmat] = computeOverlap(homoplanes)
	% Given homoplanes, compute their intersections
	% Inputs
	%  homoplanes: List of homoplanes vertices

	% Output
	%  pixelplane_overlap: all intersections (i.e points) of the regions
	%  kmat: points from the first camera region inside the second region
	%  mmat: points from the second camera region insde the first region
	if length(homoplanes) == 2
		X = poly2poly(homoplanes{1}',homoplanes{2}');
		intersect = poly2ccw(X');
		if ~isempty(intersect)
			% Points from region 1 inside region 2
		    kinside_points = {};
		    for k=1:size(homoplanes{1},1)
		        if polyin(homoplanes{1}(k,:),homoplanes{2})
		            kinside_points{end+1} = [k homoplanes{1}(k,:)];
		        end
		    end
			% Points from region 2 inside region 1
		    minside_points = {};
		    for m=1:size(homoplanes{2},1)
		        if polyin(homoplanes{2}(m,:),homoplanes{1})
		            minside_points{end+1} = [m homoplanes{2}(m,:)];
		        end
		    end
		    kmat = cell2mat(kinside_points');
		    mmat = cell2mat(minside_points');

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
	else
		% TODO Get all possible combinations of pairs of regions and compute their overlaps and store them and then compute the overlaps on them
		% combs = combnk(1:3,2);

	end
end
