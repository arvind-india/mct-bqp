function t = H_ucla(homography, point)
    x = point(1);
    y = point(2);
    A = homography.K * homography.R;
    A = [A(:,1:2) [512-x; 288-y;- 1]];

    b = homography.K * homography.R * homography.T;
    new_pts = A \ b;

    t = [new_pts(1)./new_pts(3) new_pts(2)./new_pts(3)];


    H = homography.K * [homography.R(:,1) homography.R(:,2) homography.T];
    t = inv(H) * [x; y; 1];
    t = [new_pts(1)./new_pts(3) new_pts(2)./new_pts(3)];
end
