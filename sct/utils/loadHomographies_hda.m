function [homographies, invhomographies] = loadHomographies_hda()
    setCaptureParams_hda_elevator;
    for i=1:length(cameras)
        homographies{i} = dlmread(strcat(homography_directory, '/cam', num2str(cameras{i}), 'homography.txt'));
        invhomographies{i} = inv(homographies{i});
    end
