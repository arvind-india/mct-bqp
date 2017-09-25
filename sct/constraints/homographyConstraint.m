function c_h = homographyConstraint(allbbs,allDetections,cam_id,n,k,f,homographies,invhomographies)
    %Homograph Constraint
    cam1_k = k;
    cam2_k = k;
    c_h = {};
    sigma = sqrt(sqrt(1220^2 + 720^2)); %Distance corner to corner, max distance in camera space

    %Camera Alameda
    if cam_id == 1
        cam1_n = n;
        cam2_n = size(allDetections{2}{f},1);
        for j = 1:cam1_n
            hcandidate_weights = zeros(cam1_k,1);
            for i = 1:cam1_k
                candidate_pos = allbbs{j}(i,:);
                x_i = candidate_pos(1:2);
                H_weight = 0.0;
                for m = 1:cam2_n
                    xhomog_pos = allDetections{2}{f}(m,:);
                    xhomog = [xhomog_pos(3:4) 1];
                    xhomog = xhomog*invhomographies{1}*homographies{2};
                    lambda = xhomog(3);
                    xhomog = xhomog./lambda;
                    val = exp(-sqrt((x_i(1)-xhomog(1))^2 + (x_i(2)-xhomog(2))^2)/sigma^2);
                    H_weight = H_weight + val;
                end
                hcandidate_weights(i) = H_weight/cam2_n;
            end
            c_h{end+1} = hcandidate_weights;
        end

    %Camera Central
    elseif cam_id == 2
        cam2_n = n;
        cam1_n = size(allDetections{1}{f},2);
        for j = 1:cam2_n
            hcandidate_weights = zeros(cam2_k,1);
            for i = 1:cam2_k
                candidate_pos = allbbs{j}(i,:);
                x_i = candidate_pos(1:2);
                H_weight = 0.0;
                for m = 1:cam1_n
                    xhomog_pos = allDetections{1}{f}(m,:);
                    xhomog = [xhomog_pos(3:4) 1];
                    xhomog = xhomog*invhomographies{2}*homographies{1};
                    lambda = xhomog(3);
                    xhomog = xhomog./lambda;
                    val = exp(-sqrt((x_i(1)-xhomog(1))^2 + (x_i(2)-xhomog(2))^2)/sigma^2);
                    H_weight = H_weight + val;
                end
                hcandidate_weights(i) = H_weight/cam1_n;
            end
            c_h{end+1} = hcandidate_weights;
        end

    end
