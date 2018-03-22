function H = solve_homography(pin, pout, option)
    %   takes a 2xN matrix of input vectors and
    %   a 2xN matrix of output vectors, and returns the homogeneous
    %   transformation matrix that maps the inputs to the outputs,
    if strcmp(option, 'svd')
        H_mat = homography_svd(pin', pout');
    elseif strcmp(option, 'ransac')
        pin = horzcat(pin, ones(size(pin,1),1));
        pout = horzcat(pout, ones(size(pin,1),1));
        H_mat = homography_ransac(pin', pout', 0.05);
    end
    normalization = H_mat(3,3);
    H = H_mat;
    H(:,1) = H_mat(:,1)/normalization;
    H(:,2) = H_mat(:,2)/normalization;
    H(:,3) = H_mat(:,3)/normalization;
