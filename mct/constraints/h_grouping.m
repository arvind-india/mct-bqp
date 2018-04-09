function T_o = h_grouping(N_o,targs_o,targs_in_overlap,groups_o)
    T_o = zeros(N_o*2,N_o*2);
    for i1 = 1:N_o
        for i2 = 1:N_o
            t1 = targs_o(i1,:);
            t2 = targs_o(i2,:);
            cam1 = t1(1); cam2 = t2(1);
            if cam1 == cam2
                % Get the other camera
                o_c = rem(cam1,2) + 1;
                for j1 = 1:size(targs_in_overlap{o_c},1)
                    % For the two candidates of i1
                    for j2 = 1:size(targs_in_overlap{o_c},1)
                        c1 = targs_in_overlap{o_c}(j1,:);
                        c2 = targs_in_overlap{o_c}(j2,:);
                        if c1(3) ~= c2(3)
                            % TODO Generalize
                            ec1 = 0; ec2 = 0;
                            % If c1 and t1 are in the same group
                            if groups_o(i1) == groups_o(2*(o_c-1) + c1(3))
                                ec1 = -pdist([t1;c1]);
                            end

                            % If c2 and t2 are in the same group
                            if groups_o(i2) == groups_o(2*(o_c-1) + c2(3))
                                ec2 = -pdist([t2;c2]);
                            end

                            T_o((i1-1)*2 + j1,(i2-1)*2 + j2)  = ec1 + ec2;
                        end
                    end
                end
            end
        end
    end
end
