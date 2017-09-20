global pedestrian_detector
pedestrian_detector = 'AcfInria';

global crop_directories
crop_directories = {[image_directories pedestrian_detector '/allF_cam' cameras{1} '.txt'],[image_directories pedestrian_detector '/allF_cam' cameras{2} '.txt']};

global ground_truth
ground_truth = {['~/hda_code/HDA_TRAJECTORIES/hda_data/ground_truth/' pedestrian_detector '/allG_cam' cameras{1} '.txt'],['~/hda_code/HDA_TRAJECTORIES/hda_data/ground_truth/' pedestrian_detector '/allG_cam' cameras{2} '.txt']};
