function [G, groups, dendrograms] = grouping(N, ns, k, groups, targs_percam, cands_percam, f, G_sigma, num_cams, comfort_distance, dataset, cluster_update_freq, clustering, Gamma)
    % NOTE Initializing clusters every n frames
    if rem(f-1,cluster_update_freq) == 0
        dendrograms = cell(num_cams,1);
        groups = cell(num_cams,1);
        for i = 1:num_cams
            if size(targs_percam{i},1) ~= 1
                Y = pdist(targs_percam{i}(:,8:9));
                Z = linkage(Y, clustering);
                C = cluster(Z, 'cutoff', comfort_distance, 'criterion', 'distance');
                groups{i} = C;
                dendrograms{i} = Z;
            end
        end
    end
    %---------------------------------------------------------------------------
    % NOTE grouping (enforce that people in "talking distance" groups remain together)
    T = zeros(N * k, N * k);
    for c = 1:num_cams
        for i1=1:ns(c)
            for i2=1:ns(c)
                t1 = targs_percam{c}(i1,:);
                t2 = targs_percam{c}(i2,:);
                for j1=1:k
                    for j2=1:k
                        if i1 ~= i2
                            c1 = cands_percam{c}{i1}(j1,5:6);
                            c2 = cands_percam{c}{i2}(j2,5:6);
                            dist = pdist([c1;c2],'minkowski',2); % Any minkowski distance (here its euclidean)
                            % If the targets are in the same group
                            if strcmp(dataset, 'ucla')
                                id1 = i1;
                                id2 = i2;
                            elseif strcmp(dataset, 'hda')
                                id1 = t1(3);
                                id2 = t2(3);
                            end

                            if groups{c}(id1) == groups{c}(id2)
                                e12 = pdist([t1(8:9);t2(8:9)],'minkowski',2); % Any minkowski distance (here its euclidean)
                            else
                                e12 = 0;
                            end
                            T((i1-1)*k + (c-1)*(ns(c)-1) + j1, (i2-1)*k + (c-1)*(ns(c)-1) + j2) = exp((-dist + e12)/G_sigma);
                        end
                    end
                end
            end
        end
    end

    T = Gamma * T;

    % Augment G
    t = zeros(size(T) * 2);
    t(1:size(T,1),1:size(T,2)) = T;

    Dinvsq = diag(sum(t,2)).^(-1/2); %row sum
    Dinvsq(~isfinite(Dinvsq)) = 0; %Remove infinites from Dsinvsq, Ds.^(-1/2) is only in the diagonal
    G = eye(N*k*2) - Dinvsq*t*Dinvsq; %Normalized Laplacian matrix so G is convex


end
