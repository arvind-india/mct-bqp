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
for i=[1 2 3 4]
    regions{i}.ground.convexhull = regions{i}.ground.v(convhull(regions{i}.ground.v(:,1),regions{i}.ground.v(:,2)), 1:2);
    plot(regions{i}.ground.convexhull(:,1),regions{i}.ground.convexhull(:,2), ['-' coo(i)]);
end

cameras = [1 2 4];
dets = cell(length(cameras),1);
dets{1} = [1, 0, 0, 131. 456-114, 30, 30, 0, 0; 1, 0, 0, 310, 456-114. 30, 30, 0, 0; 1, 0, 0, 472, 456-114. 30, 30, 0, 0];
dets{2} = [1, 0, 0, 382, 369-114, 30, 30, 0, 0; 1, 0, 0, 373, 383-114, 30, 30, 0, 0; 1, 0, 0, 355, 403-114, 30, 30, 0, 0];
dets{4} = [1, 0, 0, 469, 640-114, 30, 30, 0, 0; 1, 0, 0, 636, 628-114, 30, 30, 0, 0; 1, 0, 0, 784, 616-114, 30, 30, 0, 0];
for i = cameras
    %dets{i} = csvread(strcat('/home/pedro/mct-bqp/data/ucla/groundtruth/view-GL' ,num2str(cameras(i)), '.txt'));
    A = data.homographies(i).K * data.homographies(i).R;
    b = data.homographies(i).KRC;
    for j=1:size(dets{i},1)
        xx = dets{i}(j,4);
        yy = dets{i}(j,5);
        bb_width = dets{i}(j,6);
        bb_height = dets{i}(j,7);
        %xx = xx + bb_width/2;
        %yy = yy + bb_height;

        J = [A(:,1:2) [512-xx; 288-yy;-1]];
        x = J\b;

        if yy<=400
            for jjj = 0.1:0.1:0.9
                J = [A(:,1:2) [512-ii; 288-jj+jjj;-1]];
                t = J\b;
            end
        end

        dets{i}(j,8) = x(1);
        dets{i}(j,9) = x(2);
    end
    scatter(dets{i}(:,8),dets{i}(:,9),10,'MarkerEdgeColor', coo(i), 'MarkerEdgeColor', coo(i));
end
