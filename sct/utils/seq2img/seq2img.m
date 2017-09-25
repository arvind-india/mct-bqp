function seq2img(camera, end_frame)

    if ~exist('end_frame','var')
        end_frame=Inf;
    end
    addpath(genpath('toolbox/'));
    path(path,genpath('toolbox/'));
    addpath(genpath('code3.2.1/'));
    path(path,genpath('code3.2.1/'));

    input_seq_file = strcat('/home/pedro/HDA_Dataset_V1.3/hda_image_sequences_matlab/camera', num2str(camera), '.seq');
    output_directory = strcat('/home/pedro/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/rcnn/DeepPed/campus2_code/hda_data/images/cam', num2str(camera), '/');

    skip_frame = 1;
    start_frame = 0;
    sr = seqIo(input_seq_file, 'reader');
    info = sr.getinfo();
    end_frame = min(info.numFrames - 1, end_frame);
    frames = start_frame:skip_frame:end_frame;

    disp(['Converting ' num2str(end_frame-start_frame) ' frames. This may take a while...']);
    for frame = frames
        disp(frame);
        sr.seek(frame);
        I = sr.getframe();
        imwrite(I,strcat(output_directory, num2str(frame),'.png'));
        clear I;
    end;
    sr.close();
    disp('Finished.');

end
