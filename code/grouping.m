function T = grouping(N,k,groups,targs,targs_percam,cands, G_sigma)

    if rem(f,tau) == 0
        groups = cell(2,1);
        for i = 1:length(cameras)
                Y = pdist(targs_percam{i}(:,8:9));
                Z = linkage(Y,clustering);
                C = cluster(Z,'cutoff',comfort_distance,'criterion','distance');
                groups{i} = C;
        end
    end
    if rem(f,h_tau) == 0
        y_spatial = pdist(targs_o(:,8:9)); % Z_spatial is a dendrogram
        Z_spatial = linkage(y_spatial,clustering); % Use euclidean distances to do hierarchical clustering (single is default)
        groups_spatial = cluster(Z_spatial,'cutoff',h_group_distance,'criterion','distance');
    end
    %--------------------------------
    % NOTE grouping
    T = grouping(N,k,groups,targs,targs_percam,cands,G_sigma);
    Dinvsq = diag(sum(T,2)).^(-1/2); %row sum
    Dinvsq(~isfinite(Dinvsq)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
    G = eye(N*k) - Dinvsq*T*Dinvsq; %Normalized Laplacian matrix so G is convex
    %plotGrouping(T,G);
    G = zeros(N*k);
    T_spatial = h_grouping(N_o,targs_o,targs_in_overlap,groups_spatial,h_G_sigma);
    Dinvsq_spatial = diag(sum(T_spatial,2)).^(-1/2); %row sum
    Dinvsq_spatial(~isfinite(Dinvsq_spatial)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
    G_spatial = Dinvsq_spatial*T_spatial*Dinvsq_spatial; %TODO Due to its structure it cant be Normalized Laplacian matrix so G is convex
    %G_spatial = zeros(N_o*2,N_o*2);

    T = zeros(N*k,N*k);
    for i1=1:N
        for i2=1:N
            for j1=1:k
                for j2=1:k
                    t1 = targs(i1,:);
                    t2 = targs(i2,:);
                    cam1 = t1(1); cam2 = t2(1);
                    % TODO if the targets are from the same camera
                    if cam1 == cam2
                        % TODO if t1 == t2 (use target id from the detections -- should be unique)
                        if t1(3) == t2(3)
                            T((i1-1)*k + j1,(i2-1)*k + j2) = 0;
                        % TODO else
                        else
                            c1 = cands{i1}(j1,5:6); % Candidate belonging to t1
                            c2 = cands{i2}(j2,5:6); % Candidate belonging to t2
                            dist = pdist([c1;c2],'minkowski',2); % Any minkowski distance (here its euclidean)
                            % TODO if the targets are in the same group
                            if groups{cam1}(t1(3)) == groups{cam1}(t2(3))
                                % TODO within group distance
                                e12 = pdist([t1(8:9);t2(8:9)],'minkowski',2); % Any minkowski distance (here its euclidean)
                            % TODO else
                            else
                                e12 = 0;
                            end
                            T((i1-1)*k + j1,(i2-1)*k + j2) = exp((-dist + e12)/G_sigma);
                        end
                    % TODO if the targets are NOT from the same camera (do we allow groups from different cameras?)
                    else
                        T((i1-1)*k + j1,(i2-1)*k + j2) = 0;
                    % NOTE this happens if the inter-camera target coupling lets this through
                    end
                end
            end
        end
    end
end
