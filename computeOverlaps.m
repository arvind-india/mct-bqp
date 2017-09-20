function homoplanes = computeOverlap(cameraListImages, homographies, invhomographies)
    setCaptureParams_campus2;
    setDetectionParams_campus2;

    figure
    hold on
    %Alameda
    inplanes{1} = [494 545; 426 687; 602 681; 852 590; 631 539];
    %Central
    inplanes{2} = [162 510; 702 608; 917 558; 603 390; 447 412];

    homoplanes = homographyCampusTransform(inplanes,homographies,1);
    colors = {'Blue','Purple'};
    for i=1:2
        homoplanes{i} = [homoplanes{i};homoplanes{i}(1,:)];
        inplanes{i} = [inplanes{i};inplanes{i}(1,:)];
        drawPoly(homoplanes{i},colors{i},0.5,false);
        scatter(homoplanes{i}(:,1),homoplanes{i}(:,2),'MarkerFaceColor',rgb(colors{i}));
    end
