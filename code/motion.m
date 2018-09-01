function [m, targs_speed] = motion(k, targs_speed, targs_percam, cands_percam, gnd_detections, m_sigma, f, cands_clones, start_frames, num_cams, dt, detection_frames)
    m = cell(num_cams,1);
    % NOTE Initializing motion models with an heuristic
    if f == 1
        % Next detection (other than f = 1)
        next_det = detection_frames(2);
        targs_speed = cell(num_cams,1);
        for c = 1:num_cams
            future_dets = gnd_detections{c}{start_frames(c) + next_det};
            for i = 1:size(targs_percam{c},1)
                id = targs_percam{c}(i,2);
                dx = targs_percam{c}(i,8);
                dy = targs_percam{c}(i,9);
                % TODO Should use the first two detections to get initial speeds. Maybe use all averaged detections
                % TODO Use dt
                for d = 1:size(future_dets,1)
                    if id == future_dets(d,2)
                        initial_speed_x = (future_dets(d,8) - dx);
                        initial_speed_y = (future_dets(d,9) - dy);
                        %initial_speed_x = 0
                        %initial_speed_y = 0
                        targs_speed{c}{i} = [initial_speed_x; initial_speed_y];
                    end
                end
            end
        end
    end
    %---------------------------------------------------------------------------
    % NOTE motion
    for c = 1:num_cams
        c_m = cell(size(targs_percam{c},1),1);
        c_cm = cell(size(targs_percam{c},1),1);
        for i = 1:size(targs_percam{c},1)
            gaussian_center = [targs_percam{c}(i,8) + targs_speed{c}{i}(1) ; targs_percam{c}(i,9) + targs_speed{c}{i}(2) ];
            candidate_gaussian_weights = zeros(k,1);
            for j = 1:k
                u = cands_percam{c}{i}(j,5:6);
                exponent = ((u(1)-gaussian_center(1))^2)/(2*m_sigma{c}.^2)+((u(2)-gaussian_center(2))^2)/(2*m_sigma{c}.^2);
                candidate_gaussian_weights(j) = 1*exp(-exponent);
            end
            c_m{i} = - candidate_gaussian_weights;

            candidates_clones = cands_clones{c}{i}(:,5:6);
            clone_responses = ones(k, 1) * 9999999;
            s = 0;
            for j=1:k
                if candidates_clones(j,1) ~= 0
                    p1 = targs_percam{c}(i,8:9);
                    p2 = candidates_clones(j,:);
                    v1 = targs_speed{c}{i};
                    v2 = targs_speed{getOtherCamIdx(c)}{j};
                    clone_responses(j) = sum(abs(v1 - v2)) + sqrt((p1(1)-p2(1))^2 + (p1(2)-p2(2))^2);
                    s = s + clone_responses(j);
                end
            end
            % TODO Check this
            c_cm{i} = clone_responses./s;
        end
        m{c} = [cell2mat(c_m) ; cell2mat(c_cm)];
        % TODO Recursive filter (use weights)
    end
end


function oi = getOtherCamIdx(idx)
  oi = rem(idx,2) + 1; % TODO This is a dirty hack to get 1 if 2 or 2 if 1
end
