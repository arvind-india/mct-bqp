function plotDetectionsGroundPlane_hda(allDetections,homographies)
    setCaptureParams_hda_elevator;
    for i=1:length(cameras)
        pedpos = horzcat(allDetections{i}(:,3)+0.5*allDetections{i}(:,5), allDetections{i}(:,4)+0.5*allDetections{i}(:,6));
        transfpos = zeros(size(pedpos,1),2);
        for j=1:size(pedpos,1)
            pts = pedpos(j,:);
            u = pts(1);
            v = pts(2);
            o = 1;
            uvo = [u; v; o];
            new_pts = homographies{i}*uvo;

            new_pts = [new_pts(1)./new_pts(3) new_pts(2)./new_pts(3)];
            transfpos(j,:) = new_pts;
        end
        scatter(transfpos(:,1),transfpos(:,2),4,'filled','MarkerFaceColor',rgb('LightGray'));
    end
