function homoplanes = computeHomoplanes(inplanes, homographies, num_cameras, dataset)
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
