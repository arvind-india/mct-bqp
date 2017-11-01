function allPedIds = computeAllPedIds(homographies, allPedIds, cameras)
    % Generic 2d gaussian function without cross-correlations (circular gaussian)
    % Inputs
    %  gsize: gridsize for the gaussian evaluation
    %  sigma: correlation matrix
    %  center: mean

    % Output
    %  mat: matrix of the distribution

    for id=1:length(cameras)
        for p=1:size(allPedIds{id},1)
            if ~isempty(allPedIds{id}{p})
                pedpos = horzcat(allPedIds{id}{p}(:,3) + allPedIds{id}{p}(:,5)/2, allPedIds{id}{p}(:,4) + allPedIds{id}{p}(:,6));
                transfpos = zeros(size(pedpos,1),2);
                for i=1:size(pedpos,1)
                    pts = pedpos(i,:);
                    u = pts(1);
                    v = pts(2);
                    o = 1;
                    uvo = [u; v; o];
                    new_pts = homographies{id}*uvo;

                    new_pts = [new_pts(1)./new_pts(3) new_pts(2)./new_pts(3)];
                    transfpos(i,:) = new_pts;
                end
                allPedIds{id}{p}(:,9:10) = transfpos;
            end
        end
    end
