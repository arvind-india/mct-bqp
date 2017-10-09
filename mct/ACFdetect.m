function allDetections = ACFdetect()
    % Get detections from the HDA set that were performed with ACF
    % Outputs
    %  allDetections: all pedestrians identified by the detected

    setCaptureParams_hda_elevator; % Has camera info
    setDetectionParams_hda_elevator; % has detector info

    allDetections = cell(length(cameras),1); % Allocate

    for idx_c = 1:length(cameras)
        cam = cameras{idx_c};
        F = dlmread(char(crops{idx_c})); % Load data from file
        G = dlmread(char(ground_truth{idx_c})); % Load data from file
        F_xlen = size(F,1); % G_xlen should be the same
        F_ylen = size(F,2); G_ylen = size(G,2);
        allDetections{idx_c} = zeros(F_xlen, F_ylen + (G_ylen-2)); % Allocate, the first 2 columns of G have redundant info
        for k = 1:F_xlen
            G_line = G(k,:);
            F_line = F(k,:);
            frame = F_line(2);
            allDetections{idx_c}(k,:) = horzcat(F_line, G_line(3:end));
        end
    end
