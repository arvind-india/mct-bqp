global pedestrian_detector
pedestrian_detector = 'AcfInria';

global crops
crops = {strcat(image_directories, pedestrian_detector, '/allF_cam', int2str(cameras{1}), '.txt'),
        strcat(image_directories, pedestrian_detector, '/allF_cam', int2str(cameras{2}), '.txt')};

global ground_truth
ground_truth = {strcat('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/ground_truth/', pedestrian_detector, '/allG_cam', int2str(cameras{1}), '.txt'),
                strcat('~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/ground_truth/', pedestrian_detector, '/allG_cam', int2str(cameras{2}), '.txt')};
