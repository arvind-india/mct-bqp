function cameraListImages = loadImages(cameras, image_directory, num_frames, start_frame, dataset)
    if strcmp(dataset, 'hda')
        for i=1:length(cameras)
            direct = strcat(image_directory, num2str(cameras{i}));
            if exist(direct{1},'dir')
                foldercontent = dir(direct{1});
                if numel(foldercontent) == 2
                    % folder exists and is empty
                    seq2img(cameras{i},start_frame(i), num_frames(i));
                end
            end
            %cameraListImages{i} = cell(sample_size,1);
            for j=start_frame(i):(start_frame(i) + num_frames(i))
                cameraListImages{i}{j} = imread(strcat(direct{1}, '/', num2str(j), '.png'));
            end
        end
    elseif strcmp(dataset, 'campus2')
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
    end
