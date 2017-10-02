function plotDetectionsGroundPlane_hda(allDetections,homographies, ground_plane_regions, show_outliers)
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
        if strcmp(show_outliers, 'show_outliers') == 1
            for n = 1:size(transfpos,1)
                region_counter = 0;
                for i=1:length(ground_plane_regions)
                    if inpoly(transfpos(n,:),ground_plane_regions{i})
                        region_counter = region_counter + 1;
                    end
                end
                if region_counter ~= 0
                    scatter(transfpos(n,1),transfpos(n,2),4,'filled','MarkerFaceColor',rgb('LightGray'));
                else
                    scatter(transfpos(n,1),transfpos(n,2),4,'filled','MarkerFaceColor',rgb('Salmon'));
                end
            end
        else
            scatter(transfpos(:,1),transfpos(:,2),4,'filled','MarkerFaceColor',rgb('LightGray'));
        end
    end
