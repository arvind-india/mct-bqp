function [assignments, P] = solve_assignment(Scores, test_targets, previous_test_targets, cameraListImages, assignmentAlgo)
    if strcmp(assignmentAlgo, 'bertsekas') == 1
        NOT_POSSIBLE = 0; % Means this is impossible in the implementation
    elseif (strcmp(assignmentAlgo, 'jonker_volgenant') == 1) || (strcmp(assignmentAlgo, 'munkres') == 1)
        NOT_POSSIBLE = intmax - 1;
    end
    for a=1:size(Scores,1)
        for b=1:size(Scores,2)
            if a == b % Same target
                Scores(a,b) = NOT_POSSIBLE;
            else
                % Belonging to the same camera
                if test_targets(a,1) == test_targets(b,1)
                    Scores(a,b) = NOT_POSSIBLE;
                else
                    % Compute score

                    % TODO Generalize this stuff


                    targ1 = test_targets(a,:); targ2 = test_targets(b,:);
                    prev_targ1 = previous_test_targets(a,:); prev_targ2 = previous_test_targets(b,:);
                    p1 = targ1(9:10);
                    p2 = targ2(9:10);
                    pos_score = abs(p1 - p2);
                    v1 = (p1 - prev_targ1(9:10))/(1/fps);
                    v2 = (p2 - prev_targ2(9:10))/(1/fps);
                    vel_score = abs(v1 - v2);
                    img1 = imcrop(cameraListImages{1}{targ1(2)},targ1(3:6));
                    I1 = [imhist(img1(:,:,1)) imhist(img1(:,:,2)) imhist(img1(:,:,3))];
                    img2 = imcrop(cameraListImages{2}{targ2(2)},targ1(3:6));

                    I2 = [imhist(img2(:,:,1)) imhist(img2(:,:,2)) imhist(img2(:,:,3))];
                    appearance_score = abs(I1 - I2);
                    if strcmp(assignmentAlgo, 'bertsekas') == 1
                        % Inverse since the algorithm tries to bind the maximum
                        Scores(a,b) = 1/(sum(pos_score) + sum(vel_score) + (1/3)*sum(appearance_score(:,1),1)/(256*max(appearance_score(:,1))) + (1/3)*sum(appearance_score(:,2),1)/(256*max(appearance_score(:,2))) + (1/3)* sum(appearance_score(:,3),1)/(256*max(appearance_score(:,3))));
                    elseif (strcmp(assignmentAlgo, 'jonker_volgenant') == 1) || (strcmp(assignmentAlgo, 'munkres') == 1)
                        Scores(a,b) = (sum(pos_score) + sum(vel_score) + (1/3)*sum(appearance_score(:,1),1)/(256*max(appearance_score(:,1))) + (1/3)*sum(appearance_score(:,2),1)/(256*max(appearance_score(:,2))) + (1/3)* sum(appearance_score(:,3),1)/(256*max(appearance_score(:,3))));
                    end
                end
            end
        end
    end
    if strcmp(assignmentAlgo,'bertsekas') == 1 % This algorithm has the same complexity as the Hungarian/Munkres algorithm, but avg time is much better
        % Compile mex if not already compiled
        if ~exist('utils/fastAuction_v2.6/auctionAlgorithmSparseMex.mexa64', 'file')
            mex -largeArrayDims auctionAlgorithmSparseMex.cpp -lut
        end
        Scores = sparse(Scores);
        % scale A such that round(Ascaled) has sufficient accuracy
        scalingFactor = 10 ^ 6;
        Scores_scaled = Scores * scalingFactor;
        % solve assignment problem
        tic
            [assignments, P] = sparseAssignmentProblemAuctionAlgorithm(Scores_scaled);
        time = toc;
    elseif strcmp(assignmentAlgo,'jonker_volgenant') == 1 % This algorithm is ~10 times faster than the Munkres algorithm
        resolution = eps; % NOTE This can be changed to accelerate the algorithm
        tic
            [assignments, P] = lapjv(Scores,resolution);
        time = toc;
    elseif strcmp(assignmentAlgo,'munkres') == 1
        tic
            [assignments, P] = munkres(Scores);
        time = toc;
    end
    disp(['Assignment problem algorithm took ' sprintf('%f',time) ' to complete.']);
