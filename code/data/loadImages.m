function cameraListImages = loadImages(image_directories, durations, start_frame, num_cams, dataset)
    cameraListImages = cell(num_cams,1);
    for i=1:num_cams
        image_directory = image_directories{i};
        num_frames = durations(i);
        cameraListImages{i} = {};
        if strcmp(dataset, 'hda')
            for j=start_frame:(start_frame + num_frames)
                cameraListImages{i}{j+1} = strcat(image_directory, '/', num2str(j), '.png');
            end
        elseif strcmp(dataset, 'ucla')
            for j=start_frame:(start_frame + num_frames)
                cameraListImages{i}{j+1} = strcat(image_directory, 'frame', num2str(j), '.jpg');
            end
        end
    end
end
