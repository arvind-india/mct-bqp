function c_m = motion(n,k,motion_models,cands_percam,fps, m_sigma)


    if ismember(f,detection_frames)

        motion_models = cell(N,1);
        for t = 1:size(targs,1)
            % TODO Should use the first two detections to get initial speeds. Maybe use all averaged detections
            cam = targs(t,1);
            initial_speed_x = sum(gnd_detections{cam}{start_frames(cam) + detection_frames(2)}(:,8) - gnd_detections{cam}{start_frames(cam) + detection_frames(1)}(:,8)./dt);
            initial_speed_y = sum(gnd_detections{cam}{start_frames(cam) + detection_frames(2)}(:,9) - gnd_detections{cam}{start_frames(cam) + detection_frames(1)}(:,9)./dt);
            % Assign the new models
            motion_models{t} = [targs(t,8); targs(t,9); initial_speed_x; initial_speed_y];
        end
    end
    %--------------------------------
    % NOTE motion

    m = cell(length(cameras),1);
    for i = 1:length(cameras)
        n = size(targs_percam{i},1);
        c_m = motion(n,k,motion_models,cands_percam{i},fps,m_sigma{i});
        m{i} = c_m;
        %plotMotion(i, c_m, k, n, floor_image, cands_percam);
    end
    m = normalize(m,k,N); % NOTE Normalize across each candidate!

    %--------------------------------
    % NOTE motion spatial and appearance spatial
    m_spatial = cell(length(cameras),1);
    for i = 1:length(cameras)
        cands_spatial = cell(n_o{i},1);
        o_c = rem(i,2) + 1; % TODO This is a dirty hack
        c_spatial = targs_in_overlap{o_c};
        for j = 1:n_o{i}
            cands_spatial{j} = c_spatial(:,4:9);
        end
        % TODO use inter-frame appearance models that have already been built Z_model and y_models
        [c_a_spatial, w_spatial] = appearance(n_o{o_c},n_o{i},cands_spatial,images{o_c},appearance_method,h_lambda,weights_spatial{i},filter,Z_models,y_models);
        a_spatial{i} = c_a_spatial;
        weights_spatial{i} = w_spatial; % TODO Store weights which might be useful
        c_m_spatial = motion(n_o{i},n_o{o_c},motion_models,cands_spatial,fps,h_m_sigma{i});
        m_spatial{i} = c_m_spatial;
    end
    a_spatial = cell2mat(a_spatial); m_spatial = cell2mat(m_spatial);

    % TODO This is experimental
    a_spatial = normalize(a_spatial,2,N_o); m_spatial = normalize(m_spatial,2,N_o);

    c_m = cell(n,1);
    dt = 1.0/fps;
    %A = [1 dt; 1 dt];
    for i = 1:n
        gaussian_center = [motion_models{i}(1) + dt  * motion_models{i}(3) ; motion_models{i}(2) + dt * motion_models{i}(4)];
        candidate_gaussian_weights = zeros(k,1);
        for j = 1:k
            u = cands_percam{i}(j,5:6);
            exponent = ((u(1)-gaussian_center(1))^2)/(2*m_sigma(1,1).^2)+((u(2)-gaussian_center(2))^2)/(2*m_sigma(2,2).^2);
            candidate_gaussian_weights(j) = -1*exp(-exponent);
        end
        c_m{i} = candidate_gaussian_weights;
    end
    c_m = cell2mat(c_m);
end
