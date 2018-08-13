function [m, targs_speed] = motion(k, targs_speed, targs_percam, cands_percam, gnd_detections, m_sigma, f, cands_clones, start_frames, num_cams, dt)
    m = cell(num_cams,1);
    % NOTE Initializing motion models with an heuristic
    if f == 1
        targs_speed = cell(num_cams,1);
        jump = 4; % Look ahead 4 frames
        for c = 1:num_cams
            for i = 1:size(targs_percam{c},1)
                % TODO Should use the first two detections to get initial speeds. Maybe use all averaged detections
                initial_speed_x = sum(gnd_detections{c}{start_frames(c) + jump}(:,8) - gnd_detections{c}{start_frames(c) + f}(:,8)./dt);
                initial_speed_y = sum(gnd_detections{c}{start_frames(c) + jump}(:,9) - gnd_detections{c}{start_frames(c) + f}(:,9)./dt);
                % initial_speed_x = 10.0;
                % initial_speed_y = 0;
                % Assign the new models
                targs_speed{c}{i} = [initial_speed_x; initial_speed_y];
            end
        end
    end
    %---------------------------------------------------------------------------
    % NOTE motion
    for c = 1:num_cams
        c_m = cell(size(targs_percam{c},1),1);
        c_cm = cell(size(targs_percam{c},1),1);
        for i = 1:size(targs_percam{c},1)
            gaussian_center = [targs_percam{c}(i,8) + dt * targs_speed{c}{i}(1) ; targs_percam{c}(i,9) + dt * targs_speed{c}{i}(2) ];
            candidate_gaussian_weights = zeros(k,1);
            for j = 1:k
                u = cands_percam{c}{i}(j,5:6);
                exponent = ((u(1)-gaussian_center(1))^2)/(2*m_sigma{c}.^2)+((u(2)-gaussian_center(2))^2)/(2*m_sigma{c}.^2);
                candidate_gaussian_weights(j) = -1*exp(-exponent);
            end
            c_m{i} = - candidate_gaussian_weights;

            candidates_clones = cands_clones{c}{i}(:,5:6);
            clone_responses = zeros(k, 1);
            for j=1:k
                if candidates_clones(j,1) ~= 0
                    p1 = targs_percam{c}(i,8:9);
                    p2 = candidates_clones(j,:);
                    v1 = targs_speed{c}{i};
                    v2 = targs_speed{getOtherCamIdx(c)}{j};
                    clone_responses(j) = sum(abs(v1 - v2)) + sqrt((p1(1)-p2(1))^2 + (p1(2)-p2(2))^2);
                end
            end
            c_cm{i} = clone_responses;


        end
        m{c} = [cell2mat(c_m) ; cell2mat(c_cm)];
        % TODO Recursive filter (use weights)
    end
end


function oi = getOtherCamIdx(idx)
  oi = rem(idx,2) + 1; % TODO This is a dirty hack to get 1 if 2 or 2 if 1
end
