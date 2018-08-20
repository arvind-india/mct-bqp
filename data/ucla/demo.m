clc
clear all
close all
planecount = 1;

%%
load('./Parking_1.mat')
for i=1:4
    ply{i}.v = zeros(frames.image{1}.imsize(1)*frames.image{1}.imsize(2)*5,3);
    ply{i}.rgb = zeros(frames.image{1}.imsize(1)*frames.image{1}.imsize(2)*5,3);
end
plycount=zeros(1,4);

for i = [1 2 3 4]
    i  
    load(frames.image{i}.label);
    labelmap = imresize(BWtotal,frames.image{i}.imsize(1:2),'nearest');
    % Point initialize
    % ground   -- 0
    frames.image{i}.ground.v = zeros(frames.image{1}.imsize(1)*frames.image{1}.imsize(2)*20,3);
    frames.image{i}.ground.rgb = zeros(frames.image{1}.imsize(1)*frames.image{1}.imsize(2)*20,3);
    frames.image{i}.ground.ptcnt = 0;
    %
    for ii=1:1:frames.image{i}.imsize(2)
        ii
        for jj=frames.image{i}.imsize(1):-1:1      
             if labelmap(jj,ii)==0 %ground
                A = frames.image{i}.K*frames.image{i}.R;
                A = [A(:,1:2) [512-ii; 288-jj;-1]];
                b = frames.image{i}.KRC;
                x = A\b;                                
                if jj == frames.image{i}.imsize(1)
                   planesign =  [x(1) x(2) 0 1]*frames.image{i}.plane{planecount}.results.Theta(1:4);
                else
                   if [x(1) x(2) 0 1]*frames.image{i}.plane{planecount}.results.Theta(1:4)*planesign < 0 % we dont draw ground after the wall
                       continue;
                   end
                end
                plycount(i) = plycount(i) + 1;
                ply{i}.v(plycount(i),:) = [x(1) x(2) 0];
                ply{i}.rgb(plycount(i),:) =  [frames.image{i}.image(jj,ii,1) frames.image{i}.image(jj,ii,2) frames.image{i}.image(jj,ii,3)];       
                % Add to ground point set
                frames.image{i}.ground.ptcnt = frames.image{i}.ground.ptcnt+1;
                frames.image{i}.ground.v(frames.image{i}.ground.ptcnt,:) = ply{i}.v(plycount(i),:); 
                frames.image{i}.ground.rgb(frames.image{i}.ground.ptcnt,:) = ply{i}.rgb(plycount(i),:);
                
                if jj<=400
                    for jjj = 0.1:0.1:0.9
                        A = frames.image{i}.K*frames.image{i}.R;
                        A = [A(:,1:2) [512-ii; 288-jj+jjj;-1]];
                        b = frames.image{i}.KRC;
                        x = A\b;      
                        plycount(i) = plycount(i) + 1;
                        ply{i}.v(plycount(i),:) = [x(1) x(2) 0];
                        ply{i}.rgb(plycount(i),:) =  [frames.image{i}.image(jj,ii,1) frames.image{i}.image(jj,ii,2) frames.image{i}.image(jj,ii,3)]*(1-jjj) ...
                            + [frames.image{i}.image(jj-1,ii,1) frames.image{i}.image(jj-1,ii,2) frames.image{i}.image(jj-1,ii,3)]*jjj ;
                        frames.image{i}.ground.ptcnt = frames.image{i}.ground.ptcnt+1;
                        frames.image{i}.ground.v(frames.image{i}.ground.ptcnt,:) = ply{i}.v(plycount(i),:);
                        frames.image{i}.ground.rgb(frames.image{i}.ground.ptcnt,:) = ply{i}.rgb(plycount(i),:);
                    end   
                end 
             end
        end
    end
    % shrink the pointset 
    frames.image{i}.ground.v = frames.image{i}.ground.v(1:frames.image{i}.ground.ptcnt,:);
    frames.image{i}.ground.rgb = frames.image{i}.ground.rgb(1:frames.image{i}.ground.ptcnt,:);   
end
%% Compute Convex hull on the ground
coo = ['rgbc'];
figure(1),
hold on
for i=[1 2 3 4]
    frames.image{i}.ground.convexidx = convhull(frames.image{i}.ground.v(:,1),frames.image{i}.ground.v(:,2));
    frames.image{i}.ground.convexhull = frames.image{i}.ground.v(frames.image{i}.ground.convexidx,1:2);
    plot(frames.image{i}.ground.convexhull(:,1),frames.image{i}.ground.convexhull(:,2),['-' coo(i)]);
end

%
%% Merge Ground Plane
GroundPly.v = zeros(frames.image{1}.ground.ptcnt + frames.image{2}.ground.ptcnt + frames.image{3}.ground.ptcnt + frames.image{4}.ground.ptcnt,3);
GroundPly.rgb = zeros(frames.image{1}.ground.ptcnt + frames.image{2}.ground.ptcnt + frames.image{3}.ground.ptcnt + frames.image{4}.ground.ptcnt,3);
% Add GL1
GroundPly.v(1:frames.image{1}.ground.ptcnt,:) = frames.image{1}.ground.v;
GroundPly.rgb(1:frames.image{1}.ground.ptcnt,:) = frames.image{1}.ground.rgb;
Groundcnt = frames.image{1}.ground.ptcnt;
% Add GL2
[in1,on1] = inpolygon(frames.image{2}.ground.v(:,1),frames.image{2}.ground.v(:,2),frames.image{1}.ground.convexhull(:,1),frames.image{1}.ground.convexhull(:,2));
mergeall{2} = in1 | on1;
ptcnt(2) = sum(~mergeall{2});

% Add GL5
[in1,on1] = inpolygon(frames.image{3}.ground.v(:,1),frames.image{3}.ground.v(:,2),frames.image{1}.ground.convexhull(:,1),frames.image{1}.ground.convexhull(:,2));
[in3,on3] = inpolygon(frames.image{3}.ground.v(:,1),frames.image{3}.ground.v(:,2),frames.image{2}.ground.convexhull(:,1),frames.image{2}.ground.convexhull(:,2));
mergeall{3} = in1 | on1 | in3 | on3;
ptcnt(3) = sum(~mergeall{3});

% Add GL6
[in1,on1] = inpolygon(frames.image{4}.ground.v(:,1),frames.image{4}.ground.v(:,2),frames.image{1}.ground.convexhull(:,1),frames.image{1}.ground.convexhull(:,2));
[in3,on3] = inpolygon(frames.image{4}.ground.v(:,1),frames.image{4}.ground.v(:,2),frames.image{3}.ground.convexhull(:,1),frames.image{3}.ground.convexhull(:,2));
[in2,on2] = inpolygon(frames.image{4}.ground.v(:,1),frames.image{4}.ground.v(:,2),frames.image{2}.ground.convexhull(:,1),frames.image{2}.ground.convexhull(:,2));
mergeall{4} = in1 | on1 | in3 | on3 | in2 | on2 ;
ptcnt(4) = sum(~mergeall{4});

for i = [2 3 4]
    GroundPly.v(Groundcnt+1:Groundcnt+ptcnt(i),:) = frames.image{i}.ground.v(~mergeall{i},:);
    GroundPly.rgb(Groundcnt+1:Groundcnt+ptcnt(i),:) = frames.image{i}.ground.rgb(~mergeall{i},:);
    Groundcnt = Groundcnt + ptcnt(i);
end

GroundPly.v = GroundPly.v(1:Groundcnt,:);
GroundPly.rgb = GroundPly.rgb(1:Groundcnt,:);

%%
figure(2),
showPointCloud(GroundPly.v,GroundPly.rgb/255);








