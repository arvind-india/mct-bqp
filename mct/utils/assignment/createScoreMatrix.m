function [S, A, P, V] = createScoreMatrix(f,n_ov1,n_ov2,targs_overlap,images,d1_metric,d256_metric, motion_models_overlap)
    S = zeros(n_ov1,n_ov2); A = zeros(n_ov1,n_ov2);
    P = zeros(n_ov1,n_ov2); V = zeros(n_ov1,n_ov2);

    for a=1:n_ov1
        for b=1:n_ov2

            % TODO Euclidean Distance
            p1 = targs_overlap{1}(a,8:9);
            p2 = targs_overlap{2}(b,8:9);
            P(a,b) = pdist([p1;p2],d1_metric);

            % TODO Euclidean distance between motion cues
            if f == 1
                v1 = 0;
                v2 = 0;
            else
                v1 = motion_models_overlap{1}(a,4:5);
                v2 = motion_models_overlap{2}(b,4:5);
            end
            V(a,b) = sum(abs(v1 - v2))/sum(abs(v1 + v2));

            % TODO Appearance
            bb_img1 = imcrop(images{1},targs_overlap{1}(a,4:7));
            bb_img2 = imcrop(images{2},targs_overlap{2}(b,4:7));
            %imshow(bb_img1);
            %waitforbuttonpress
            %imshow(cam2_img);
            %imshow(bb_img2);
            %waitforbuttonpress
            I1 = [imhist(bb_img1(:,:,1)) imhist(bb_img1(:,:,2)) imhist(bb_img1(:,:,3))];
            I2 = [imhist(bb_img2(:,:,1)) imhist(bb_img2(:,:,2)) imhist(bb_img2(:,:,3))];
            % Chi-squared between hists
            appearance_score = pdist2(I1',I2',d256_metric);
            A(a,b) = (1/3)*sum(appearance_score(:,1),1) / (256*max(appearance_score(:,1))) + (1/3)*sum(appearance_score(:,2),1)/(256*max(appearance_score(:,2))) + (1/3)* sum(appearance_score(:,3),1)/(256*max(appearance_score(:,3)));

            %TODO Ground plane mapped bounding box overlap ? ofc this performs better as homographies improve
            % Is this a good approach? Ground plane is a birds eye view so this mapping might not be correct
            % TODO Map 4 corners of target a to the ground plane
            % TODO Map 4 corners of target b to the ground plane
            % Compute area of intersection
            % Compute area of intersection

            %iou = area_intersection/area_union;
        end
    end
    %S = A + V + P;
    if f == 1
        S = (A/max(A(:))) .* (P/max(P(:)));
    else
        S = (A/max(A(:))) .* (V/max(V(:))) .* (P/max(P(:)));
    end
