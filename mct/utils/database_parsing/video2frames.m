videoname = 'match5-c0';
folder = '~/mct-bqp/epfl_data/basketball/';
video = VideoReader([folder,videoname,'.avi']);

ii = 1;
while hasFrame(video)
   img = readFrame(video);
   filename = [sprintf('%03d',ii) '.jpg'];
   fullname = fullfile(['~/mct-bqp/epfl_data/basketball/',videoname],filename);
   imwrite(img,fullname)    % Write out to a JPEG file (img1.jpg, img2.jpg, etc.)
   ii = ii+1;
end
