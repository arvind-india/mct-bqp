function c_nm = neighbourhoodMotionConstraint(n,k,f,fps,allDetections,predictions,past_observations)
    c_nm = cell(n,1);
    dt = 1.0/fps;
    A = [1 dt; 1 dt];
    if isempty(predictions)
        %First frame - there are no velocities here
        for i=1:n
            target_pos = allDetections{1}{f}(i,:);
            gaussian_sum = zeros(k,1);
            %Calculate sum of weights
            weighted_sum = 0;
            for kn=1:n
                kn_pos = allDetections{1}{f}(kn,:);
                weighted_sum = weighted_sum + exp(-sqrt( (kn_pos(3)-target_pos(3))^2 + (kn_pos(4)-target_pos(4))^2 ));
            end
            for neighbour=1:n
                %Calculate weight of neighbour
                j_pos = allDetections{1}{f}(neighbour,:);
                weight = exp(-sqrt( (j_pos(3)-target_pos(3))^2 + (j_pos(4)-target_pos(4))^2 ))/weighted_sum;
                %Calculate gaussian with this neighbour
                s = [target_pos(3) + target_pos(5)/2 0; target_pos(3) + target_pos(6)/2 0]; %velocity is 0 since this is the first frame
                gaussian_median_x = A(1,:) * s(1,:)';
                gaussian_median_y = A(2,:) * s(2,:)';
                gaussian = gauss2d([5 5], [0.6 0.6], [gaussian_median_x gaussian_median_y]);
                gaussian_vector = zeros(k,1);
                for gridx=1:sqrt(k)
                    for gridy=1:sqrt(k)
                        gaussian_vector(sqrt(k)*(gridy-1)+gridx) = gaussian(gridx,gridy);
                    end
                end
                %Calculate weighted gaussian
                weighted_gaussian = weight*gaussian_vector;
                %Add it to the sum of gaussians for i
                gaussian_sum = gaussian_sum + weighted_gaussian;
            end
            c_nm{i} = gaussian_sum;
        end
    else
        %All other frames
        for i=1:n
            target_pos = allDetections{1}{f}(i,:);
            gaussian_sum = zeros(k,1);
            %Calculate sum of weights
            weighted_sum = 0;
            for kn=1:n
                kn_pos = allDetections{1}{f}(kn,:);
                weighted_sum = weighted_sum + exp(-sqrt( (kn_pos(3)-target_pos(3))^2 + (kn_pos(4)-target_pos(4))^2 ));
            end

            for neighbour=1:n
                %Calculate weight of neighbour
                j_pos = allDetections{1}{f}(neighbour,:);
                weight = exp(-sqrt( (j_pos(3)-target_pos(3))^2 + (j_pos(4)-target_pos(4))^2 ))/weighted_sum;
                %Calculate gaussian with this neighbour
                %Compute s
                minimum_dist = Inf;
                dt = 1.0/fps;
                s = zeros(2,2);
                for p = 1:size(past_observations,1) %Find pi
                    %Find the closest (past_observations_pos + past_observations_vel*dt) and (target_pos)
                    predicted_from_past_observations = zeros(2,1);
                    predicted_from_past_observations(1) = past_observations{p}(1) + past_observations{p}(3)*dt;
                    predicted_from_past_observations(2) = past_observations{p}(2) + past_observations{p}(4)*dt;
                    distance2target = sqrt((predicted_from_past_observations(1) - target_pos(1))^2 + (predicted_from_past_observations(2) - target_pos(2))^2);
                    if distance2target < minimum_dist
                        s(1,1) = past_observations{p}(1);
                        s(2,1) = past_observations{p}(2);
                        minimum_dist = distance2target;
                    end
                end
                minimum_dist = Inf;
                for p = 1:size(past_observations,1) %Find pj'
                    %Find the closest (past_observations_pos + past_observations_vel*dt) and (target_pos)
                    predicted_from_past_observations = zeros(2,1);
                    predicted_from_past_observations(1) = past_observations{p}(1) + past_observations{p}(3)*dt;
                    predicted_from_past_observations(2) = past_observations{p}(2) + past_observations{p}(4)*dt;
                    distance2target = sqrt((predicted_from_past_observations(1) - j_pos(1))^2 + (predicted_from_past_observations(2) - j_pos(2))^2);
                    if distance2target < minimum_dist
                        s(2,1) = past_observations{p}(3);
                        s(2,2) = past_observations{p}(4);
                        minimum_dist = distance2target;
                    end
                end
                gaussian_median_x = A(1,:) * s(1,:)';
                gaussian_median_y = A(2,:) * s(2,:)';
                gaussian = gauss2d([5 5], [0.6 0.6], [gaussian_median_x gaussian_median_y]);
                gaussian_vector = zeros(k,1);
                for gridx=1:sqrt(k)
                    for gridy=1:sqrt(k)
                        gaussian_vector(sqrt(k)*(gridy-1)+gridx) = gaussian(gridx,gridy);
                    end
                end
                %Calculate weighted gaussian
                weighted_gaussian = weight*gaussian_vector;
                %Add it to the sum of gaussians for i
                gaussian_sum = gaussian_sum + weighted_gaussian;
            end
            c_nm{i} = gaussian_sum;
        end
        %Clear past_observations
    end
    c_nm = cell2mat(c_nm);
