function plotDetectionsGroundPlane(allDetections,homographies,ground_plane_regions,show_outliers,dataset)
    % Plot ground plane detections
    % Inputs
    %  allDetections: all pedestrians identified by the detected
    %  homographies: homography matrices
    %  ground_plane_regions: regions in the camera plane where detections can occur
    %  show_outliers: tag to plot outliers or not
    %  dataset: tag for the dataset

    if strcmp(dataset, 'campus_2')
        setCaptureParams_campus2;
        for id=1:2
            for f=1:size(allDetections{id},1)
                pedpos = horzcat(allDetections{id}{f}(:,3)+0.5*allDetections{id}{f}(:,5), allDetections{id}{f}(:,4)+0.5*allDetections{id}{f}(:,6));
                transfpos = zeros(size(pedpos,1),2);
                for i=1:size(pedpos,1)
                    pts = pedpos(i,:);
                    u = pts(1);
                    v = pts(2);
                    o = 1;
                    uvo = [u v o];
                    new_pts = uvo*homographies{id};

                    new_pts = [new_pts(1)./new_pts(3) new_pts(2)./new_pts(3)];
                    transfpos(i,:) = new_pts;
                end
                if strcmp(show_outliers, 'show_outliers') == 1
                    for n = 1:size(transfpos,1)
                        region_counter = 0;
                        for g=1:length(ground_plane_regions)
                            if inpoly(transfpos(n,:),ground_plane_regions{g})
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
        end
    end
    if strcmp(dataset, 'hda')
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
            % Transformation is done, now to plot
            if strcmp(show_outliers, 'show_outliers') == 1
                for n = 1:size(transfpos,1)
                    region_counter = 0;
                    for g=1:length(ground_plane_regions)
                        if inpoly(transfpos(n,:),ground_plane_regions{g})
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
    end
