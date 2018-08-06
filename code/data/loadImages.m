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



        elseif strcmp(dataset, 'campus2')
            % Campus only has 2 cams
          if start_frame == 1 % Camera 1
            dinfo = dir('~/Campus_II/frames_alameda_noon_1_6_2017');
            cameraListImages = {dinfo.name};
            cameraListImages = natsortfiles(cameraListImages);
            cameraListImages(:,1:2) = [];
            cameraListImages = strcat('~/Campus_II/frames_alameda_noon_1_6_2017', '/', cameraListImages);
          elseif start_frame == 2 % Camera 2
            dinfo = dir('~/Campus_II/frames_central_noon_1_6_2017');
            cameraListImages = {dinfo.name};
            cameraListImages = natsortfiles(cameraListImages);
            cameraListImages(:,1:2) = [];
            cameraListImages = strcat('~/Campus_II/frames_central_noon_1_6_2017', '/', cameraListImages);
          end
        elseif strcmp(dataset, 'ucla')
            for j=start_frame:(start_frame + num_frames)
                cameraListImages{j+1} = strcat(image_directory, '/frame', num2str(j), '.png');
            end
        end
    end
end
