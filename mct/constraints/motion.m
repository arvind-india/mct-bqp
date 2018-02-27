function c_m = motion(n,k,motion_models,cands_homo_percam,fps)
    c_m = cell(n,1);
    dt = 1.0/fps;
    A = [1 dt; 1 dt];
    Sigma = [100 0; 0 100];
    for i = 1:n
        gaussian_center = [A(1,1) * motion_models{i}(1) + A(1,2) * motion_models{i}(3) ...
        A(2,1) * motion_models{i}(2) + A(2,2) * motion_models{i}(4)];
        candidate_gaussian_weights = zeros(k,1);
        for j = 1:k
            x = cands_homo_percam{i}(j,1);
            y = cands_homo_percam{i}(j,2);
            xc = gaussian_center(1);
            yc = gaussian_center(2);
            exponent = ((x-xc).^2)/(2*Sigma(1,1).^2)+((y-yc).^2)/(2*Sigma(2,2).^2);
            candidate_gaussian_weights(j) = 1*exp(-exponent);
        end
        c_m{i}= candidate_gaussian_weights;
    end
    c_m = cell2mat(c_m);
end
