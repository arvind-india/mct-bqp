function H = solve_homography(pin, pout, option)
    %   takes a 2xN matrix of input vectors and
    %   a 2xN matrix of output vectors, and returns the homogeneous
    %   transformation matrix that maps the inputs to the outputs,
    if strcmp(option, 'svd')
        H = homography_svd(pin', pout');
    elseif strcmp(option, 'ransac')
        H = homography_ransac(pin, pout, 0.05);
    end
