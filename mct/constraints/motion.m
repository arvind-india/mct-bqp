function c_m = motion(n,k,motion_models,cands_percam,fps, m_sigma)
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
