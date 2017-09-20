function plotDetectionsGroundPlane(cameraListImages,allDetections,homographies)
    setCaptureParams_campus2;
    for id=1:2
        %subimage(imread(cameraListImages{id}{1}));
        for f=1:size(allDetections{id},1)
            pedpos = horzcat(allDetections{id}{f}(:,3)+0.5*allDetections{id}{f}(:,5), allDetections{id}{f}(:,4)+0.5*allDetections{id}{f}(:,6));
            transfpos = zeros(size(pedpos,1),2);
            for i=1:size(pedpos,1)
                pts = pedpos(i,:);
                u = pts(1);
                v = pts(2);
                o = 1;
                uvo = [u v 1]
                new_pts = uvo*homographies{id};

                new_pts = [new_pts(1)./new_pts(3) new_pts(2)./new_pts(3)];
                transfpos(i,:) = new_pts;
            end

            scatter(transfpos(:,1),transfpos(:,2),10,'filled','MarkerFaceColor','black');
        end
    end
