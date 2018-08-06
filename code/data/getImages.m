function [next_images, images] = getImages(num_cams, cameraListImages, f, start_frames)
    next_images = cell(num_cams,1);
    images = cell(num_cams,1);
    for i = 1:num_cams
        try
            next_images{i} = imread(cameraListImages{i}{start_frames(i)+(f+1)});
            images{i} = imread(cameraListImages{i}{start_frames(i)+(f)});
        catch ME
            fprintf('Could not find dataset. If stored in a HDD, make sure it is mounted.\n');
        end
    end
end
