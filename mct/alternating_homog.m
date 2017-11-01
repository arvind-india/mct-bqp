function [nH, pp] = alternating_homog(new_Hs, pedestrian_points, id, outlier_removal_regions, ground_plane_regions, rho_r, rho_m)
    setCaptureParams_hda_elevator;
    setDetectionParams_hda_elevator;
    nH = new_Hs;
    pp = pedestrian_points;

    for n=1:size(pp,1)
        if pedestrian_points(n,5) == cameras{swap(id)}
            xi = pedestrian_points(n,7) + pedestrian_points(n,9)/2;
            yi = pedestrian_points(n,8) + pedestrian_points(n,10);
            a = new_Hs{id} * [xi; yi; 1];
            a = a./a(3);
            pp(n,1) = a(1);
            pp(n,2) = a(2);
        end
    end
    [pin, pout] = aux_homo_debug(id, cameras, outlier_removal_regions, ground_plane_regions, pedestrian_points, rho_r, rho_m);
    tic
    nH{id} = solve_homography(pin',pout','svd');
    homography_time = toc;
    disp(strcat('Homography computation takes: ', num2str(homography_time)));
end

function otherid = swap(id)
    if id == 1
        otherid = 2;
    elseif id == 2
        otherid = 1;
    end

end
