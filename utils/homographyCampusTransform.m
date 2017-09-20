function homoplanes = homographyCampusTransform(inplanes,homographies,opt)

    homoplanes = {};
    % Homography to use for the transformation of camera regions from the camera plane to the ground plane
    if opt == 1
        for i=1:2 % For the two cameras
            homoplanes{i} = zeros(5,2);
            for p=1:5
            	pts = inplanes{i}(p,:);
            	u = pts(1);
            	v = pts(2);
            	o = 1;
            	uvo = [u v 1];
            	new_pts = uvo*homographies{i};

            	new_pts = [new_pts(1)./new_pts(3) new_pts(2)./new_pts(3)];
                homoplanes{i}(p,:) = new_pts;
            end
        end
    else
        % Homography to use for the transformation of pedestrian location from the camera plane to the ground plane
        for i=1:2 % For the two cameras
            inV = inplanes;
            u = transpose(inV(:,1));
            v = transpose(inV(:,2));
            o = ones(1,2);
            uvo = [u v o];
            xylambda = uvo*homographies{i};

            outV(:,1) = transpose(xylambda(1,:));
            outV(:,2) = transpose(xylambda(2,:));
            homoplanes{i} = outV; % Should change the name, might be confusing
        end
    end
