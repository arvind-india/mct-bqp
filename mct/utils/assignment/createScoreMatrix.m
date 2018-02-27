function [S, A, P, V] = createScoreMatrix(n_ov1,n_ov2,targs_overlap,images,d1_metric,d256_metric)
    S = zeros(n_ov1,n_ov2); A = zeros(n_ov1,n_ov2);
    P = zeros(n_ov1,n_ov2); V = zeros(n_ov1,n_ov2);

    for i11=1:n_ov1
        for i22=1:n_ov2

            % TODO Euclidean Distance
            p1 = targs_overlap{1}(n_ov1,8:9);
            p2 = targs_overlap{2}(n_ov2,8:9);
            P(a,b) = pdist([p1;p2],d1_metric);

            % TODO Euclidean distance between motion cues
            v1 = 0;
            v2 = 0;
            V(a,b) = sum(abs(v1 - v2));

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

            %TODO Ground plane mapped bounding box overlap ?
        end
    end
    %S = A + V + P;
    S = (A/max(A(:))) .* (A/max(A(:))) .* (A/max(A(:)));
