function c_m = motionConstraint(n,k,f,fps,allDetections,predictions,past_observations)
    c_m = cell(n,1);
    dt = 1.0/fps;
    A = [1 dt; 1 dt];
    if isempty(predictions)
        %First frame
        %No frame has a previous velocity or position
        for i=1:n
            target_pos = allDetections{1}{f}(i,:);
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
            c_m{i} = gaussian_vector;
        end
    else
        %All others frames
        for i=1:n
            target_pos = allDetections{1}{f}(i,:);
            %Find the closest bb in past_observations
            pos = [target_pos(3) + target_pos(5)/2; target_pos(3) + target_pos(6)/2];
            minimum_dist = Inf;
            dt = 1.0/fps;
            for p=1:size(past_observations,1)
                %Find the closest (past_observations_pos + past_observations_vel*dt) and (target_pos)
                predicted_from_past_observations = zeros(2,1);
                predicted_from_past_observations(1) = past_observations{p}(1) + past_observations{p}(3)*dt;
                predicted_from_past_observations(2) = past_observations{p}(2) + past_observations{p}(4)*dt;
                distance2target = sqrt((predicted_from_past_observations(1) - pos(1))^2 + (predicted_from_past_observations(2) - pos(2))^2);
                if distance2target < minimum_dist
                    s = [past_observations{p}(1) past_observations{p}(3) ; past_observations{p}(2) past_observations{p}(4)];
                    minimum_dist = distance2target;
                end
            end
            if minimum_dist == Inf

            else
                %We have data from a past observation, of course there may be identity switches which means the data may be noisy
                gaussian_median_x = A(1,:) * s(1,:)';
                gaussian_median_y = A(2,:) * s(2,:)';
                gaussian = gauss2d([5 5], [0.6 0.6], [gaussian_median_x gaussian_median_y]);
                gaussian_vector = zeros(k,1);
                for gridx=1:sqrt(k)
                    for gridy=1:sqrt(k)
                        gaussian_vector(sqrt(k)*(gridy-1)+gridx) = gaussian(gridx,gridy);
                    end
                end
                c_m{i} = gaussian_vector; %c_m{i} = gaussian_vector';
            end
        end

        %Clear past_observations
        %past_observations = {};
    end
    c_m = cell2mat(c_m);
