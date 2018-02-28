function T = grouping(N,k,groups,targs,targs_percam,cands_homo)
    T = zeros(N*k,N*k);
    twosigma_sq = 2*10^2;
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
                            c1 = cands_homo{i1}(j1,:); % Candidate belonging to t1
                            c2 = cands_homo{i2}(j2,:); % Candidate belonging to t2
                            dist = pdist([c1;c2],'minkowski',2);
                            % TODO if the targets are in the same group
                            if groups{cam1}(t1(3)) == groups{cam1}(t2(3))
                                % TODO within group distance
                                e12 = pdist([t1(8:9);t2(8:9)],'minkowski',2);
                            % TODO else
                            else
                                e12 = 0;
                            end
                            T((i1-1)*k + j1,(i2-1)*k + j2) = exp((-dist + e12)/twosigma_sq);
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
