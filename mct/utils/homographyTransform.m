function homoplanes = homographyTransform(inplanes,homographies,num_cameras,dataset)
    % Homography transformation given datapoints and homographies
    % Inputs
    %  inplanes: the points to be mapped
    %  homographies: 3x3 matrices representative of the homographies
    %  num_cameras: number of cameras, used for indexing
    %  dataset: tag for the dataset being evaluated

    % Output
    %  homoplanes: ground planes for each camera

    % Homography to use for the transformation of camera regions from the camera plane to the ground plane
    homoplanes = {};
    for i=1:num_cameras % For the cameras
        homoplanes{i} = zeros(size(inplanes{i},1),2);
        for p=1:size(inplanes{i},1)
        	pts = inplanes{i}(p,:);
        	u = pts(1);
        	v = pts(2);
        	o = 1;
            if strcmp(dataset,'hda') == 1
                uvo = [u; v; 1];
        	    new_pts = homographies{i}*uvo;
            end
            if strcmp(dataset, 'campus_2') == 1
                uvo = [u v 1];
            	new_pts = uvo*homographies{i};
            end
        	new_pts = [new_pts(1)./new_pts(3) new_pts(2)./new_pts(3)];
            homoplanes{i}(p,:) = new_pts;
        end
    end
