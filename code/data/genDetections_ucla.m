compute_reg = 1;
data = load('/home/pedro/mct-bqp/data/ucla/homographies/homographies.mat');
data = data.data;
ply = cell(4,1);
for i = 1:4
    xsize = data.homographies(i).imsize(1);
    ysize = data.homographies(i).imsize(2);
    %C = strsplit(data.homographies(i).label,'/');
    %data.homographies(i).label = strcat('/home/pedro/mct-bqp/data/ucla/regions/',C{3})
    ply{i}.v = zeros(xsize * ysize * 5,3);
    ply{i}.rgb = zeros(xsize * ysize * 5,3);
end
plycount = zeros(1,4);

if compute_reg == 1
    regions = cell(4,1);
    % Pick your cams
    for i = [1 2 3 4]
        xsize = data.homographies(i).imsize(1);
        ysize = data.homographies(i).imsize(2);

        load(data.homographies(i).label);
        labelmap = imresize(BWtotal, [xsize ysize],'nearest');

        regions{i}.ground.v = zeros(xsize * ysize * 20,3);
        % regions{i}.ground.rgb = zeros(xsize * ysize * 20,3);
        regions{i}.ground.ptcnt = 0;

        for ii = 1:1:ysize
            for jj = xsize:-1:1
                % 1 is invalid region, 0 is ground
                if labelmap(jj,ii) == 0
                    A = data.homographies(i).K * data.homographies(i).R;
                    A = [A(:,1:2) [512-ii; 288-jj;-1]];
                    b = data.homographies(i).KRC;
                    x = A\b;

                    plycount(i) = plycount(i) + 1;
                    ply{i}.v(plycount(i),:) = [x(1) x(2) 0];
                    % Add to ground point set
                    regions{i}.ground.ptcnt = regions{i}.ground.ptcnt+1;
                    regions{i}.ground.v(regions{i}.ground.ptcnt,:) = ply{i}.v(plycount(i),:);
                end
            end
        end
        % shrink the pointset (to its actual size instead of xsize * ysize)
        regions{i}.ground.v = regions{i}.ground.v(1:regions{i}.ground.ptcnt,:);
    end
    coo = ['rgbc'];
    figure(1),
    hold on
    for i = [1 2 3 4]
        regions{i}.ground.convexhull = regions{i}.ground.v(convhull(regions{i}.ground.v(:,1),regions{i}.ground.v(:,2)), 1:2);
        plot(regions{i}.ground.convexhull(:,1),regions{i}.ground.convexhull(:,2), ['-' coo(i)]);
        % Save region points to files
        r = regions{i}.ground.convexhull;
        csvwrite(strcat('/home/pedro/mct-bqp/data/ucla/regions/region' ,num2str(i), '.txt'), r);
    end
end

% cameras = [1 2 5];
cameras = [1 2 5 6];
dets = cell(max(cameras),1);
ratio = 1.875;
% These following are some debug marks that should correspond, unfortunately they don't :c?
% dets{1} = [1, 0, 0, 131. 456-114, 30, 30, 0, 0; 1, 0, 0, 310, 456-114. 30, 30, 0, 0; 1, 0, 0, 472, 456-114. 30, 30, 0, 0];
% dets{2} = [1, 0, 0, 382, 369-114, 30, 30, 0, 0; 1, 0, 0, 373, 383-114, 30, 30, 0, 0; 1, 0, 0, 355, 403-114, 30, 30, 0, 0];
% dets{5} = [1, 0, 0, 469, 640-114, 30, 30, 0, 0; 1, 0, 0, 636, 628-114, 30, 30, 0, 0; 1, 0, 0, 784, 616-114, 30, 30, 0, 0];
parse_original = 0; % We assume the original is given
k = 1;
for i = cameras
    % Parse
    if parse_original == 1
        dets{i} = csvread(strcat('/home/pedro/mct-bqp/data/ucla/groundtruth_original/view-GL' ,num2str(i), '.txt'));
        % NOTE Camera
        dets{i} = [i*ones(size(dets{i},1),1) dets{i}];
        indices = find((dets{i}(:,8) == 1 | dets{i}(:,9) == 1));
        dets{i}(indices,:) = [];
        % Remove useless lines (ones indicating entries ARE NOT lost or occluded)
        for j=1:size(dets{i},1)
            % NOTE Frame
            % NOTE xmin, ymin
            % NOTE bb_width, bb_height
            dets{i}(j,5) = dets{i}(j,5) - dets{i}(j,3);
            dets{i}(j,6) = dets{i}(j,6) - dets{i}(j,4);
            % NOTE Track_ID (same across all cameras)
        end
        dets{i} = dets{i}(:,1:7);
        % Save
        csvwrite(strcat('/home/pedro/mct-bqp/data/ucla/groundtruth_parsed/view-GL' ,num2str(i), '.txt'), dets{i});
    else
        dets{i} = csvread(strcat('/home/pedro/mct-bqp/data/ucla/groundtruth_parsed/view-GL' ,num2str(i), '.txt'));
        A = data.homographies(k).K * data.homographies(k).R;
        b = data.homographies(k).KRC;

        lb = load(data.homographies(k).label);
        xsize = data.homographies(k).imsize(1);
        ysize = data.homographies(k).imsize(2);
        labelmap = imresize(lb.BWtotal, [xsize ysize],'nearest');

        dets{i} = [dets{i} zeros(size(dets{i},1),1) zeros(size(dets{i},1),1)];

        for j=1:size(dets{i},1)

            xx = floor(dets{i}(j,3)/ratio);
            yy = floor(dets{i}(j,4)/ratio);
            dets{i}(j,5) = dets{i}(j,5)/ratio;
            dets{i}(j,6) = dets{i}(j,6)/ratio;
            bb_width = dets{i}(j,5);
            bb_height = dets{i}(j,6);
            xx = xx + floor(bb_width/2);
            yy = yy + floor(bb_height);
            if xx >= 1024
                xx = 1023;
            end
            if yy >= 576
                yy = 576;
            end
            if labelmap(yy+1,xx+1) == 0
                J = [A(:,1:2) [512-xx; 288-yy;-1]];
                x = J\b;

                %if yy<=400
                %    for jjj = 0.1:0.1:0.9
                %        J = [A(:,1:2) [512-ii; 288-jj+jjj;-1]];
                %        t = J\b;
                %    end
                %end
                dets{i}(j,3) = xx;
                dets{i}(j,4) = yy;

                dets{i}(j,8) = x(1);
                dets{i}(j,9) = x(2);
            else
                dets{i}(j,8) = 0;
                dets{i}(j,9) = 0;
            end
        end
        dets{i}(:,7) = dets{i}(:,7) + 1;
        scatter(dets{i}(:,8),dets{i}(:,9),10,'MarkerEdgeColor', coo(k), 'MarkerEdgeColor', coo(k));
        dets{i} = (accumarray(dets{i}(:,7),(1:size(dets{i},1)).',[],@(x){dets{i}(x,:)},{}));
        k = k + 1;
    end
end
