function [pin, pout] = aux_homo_debug(id, cameras, outlier_removal_regions, ground_plane_regions, pedestrian_points, rho_r, rho_m)
    % --------------------------------------------------------------------------
    pin = {}; pout = {};
    % Get the pedestrian_points for this camera
    if id == 2
        for reps = 1:rho_m
            for n=1:size(pedestrian_points,1)
                if pedestrian_points(n,5) == cameras{id}
                    xi = pedestrian_points(n,7) + pedestrian_points(n,9)/2;
                    yi = pedestrian_points(n,8) + pedestrian_points(n,10);
                    xi_ = pedestrian_points(n-1,1);
                    yi_ = pedestrian_points(n-1,2);
                    pin{end+1} = [xi yi];
                    pout{end+1} = [xi_ yi_];
                end
            end
        end
        % --------------------------------------------------------------------------
        % Get the regions, we want these to be mapped as well
        region_points = outlier_removal_regions{id}(:,1:2);
        region_points_ = ground_plane_regions{id}(:,1:2);
        % NOTE Additional repeated points, A. Bernardino suggestion
        for reps = 1:rho_r
            for r=1:size(region_points,1)
                pin{end+1} = region_points(r,:); % xi,yi
                pout{end+1} = region_points_(r,:); % xi_, yi_
            end
        end
    elseif id == 1
        for reps = 1:rho_m
            for n=1:size(pedestrian_points,1)
                if pedestrian_points(n,5) == cameras{id}
                    xi = pedestrian_points(n,7) + pedestrian_points(n,9)/2;
                    yi = pedestrian_points(n,8) + pedestrian_points(n,10);
                    xi_ = pedestrian_points(n+1,1);
                    yi_ = pedestrian_points(n+1,2);
                    pin{end+1} = [xi yi];
                    pout{end+1} = [xi_ yi_];
                end
            end
        end
        % --------------------------------------------------------------------------
        % Get the regions, we want these to be mapped as well
        region_points = outlier_removal_regions{id}(:,1:2);
        region_points_ = ground_plane_regions{id}(:,1:2);
        % NOTE Additional repeated points, A. Bernardino suggestion
        for reps = 1:rho_r
            for r=1:size(region_points,1)
                pin{end+1} = region_points(r,:); % xi,yi
                pout{end+1} = region_points_(r,:); % xi_, yi_
            end
        end
    end
    pin = cell2mat(pin');
    pout = cell2mat(pout');
