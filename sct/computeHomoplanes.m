function homoplanes = computeHomoplanes(inplanes, homographies, dataset)
    % Given a list of points representing regions put them in the ground plane
    % Inputs
    %  homographies: the homographies for each camera
    %  inplanes: representative planes of each camera

    % Output
    %  homoplanes: ground planes for each camera

    setCaptureParams_campus2;
    setDetectionParams_campus2;

    if strcmp(dataset,'hda') == 1
        homoplanes = homographyHDATransform(inplanes,homographies,1);
    end
    if strcmp(dataset,'campus_2') == 1
        homoplanes = homographyCampusTransform(inplanes,homographies,1);
    end
    for i=1:2
        homoplanes{i} = [homoplanes{i};homoplanes{i}(1,:)];
        inplanes{i} = [inplanes{i};inplanes{i}(1,:)];
    end
