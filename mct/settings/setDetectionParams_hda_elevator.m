global pedestrian_detector
pedestrian_detector = 'AcfInria';

global crops
crops = {strcat(crops_directories, pedestrian_detector, '/allF_cam', int2str(cameras{1}), '.txt'), strcat(crops_directories, pedestrian_detector, '/allF_cam', int2str(cameras{2}), '.txt')};

global ground_truth
ground_truth = {strcat('~/mct-bqp/hda_data/ground_truth/', pedestrian_detector, '/allG_cam', int2str(cameras{1}), '.txt'), strcat('~/mct-bqp/hda_data/ground_truth/', pedestrian_detector, '/allG_cam', int2str(cameras{2}), '.txt')};

global visibility_regions_directory
visibility_regions_directory = '~/mct-bqp/hda_data/homographies/visibility_points_image_';

global floor_image
floor_image = '~/mct-bqp/hda_data/images/7th_floor_ground_plane_reference_frame_map.png';

global elevator_patio
elevator_patio = [198,167,100,100];
