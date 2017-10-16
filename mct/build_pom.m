function pom = build_pom(allDetections, cameraListImages, gplane)
    ref_img = imread('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/ref_58.png');
    test_img = imread('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/test_img_58.png');
    test_frame = 450;
    %[img_diff, bin_img] = convertBB2bin_img(allDetections{2}{test_frame}, test_img, ref_img);
    %figure
    %imshow(out);
    %figure
    %imshow(bin_img);
