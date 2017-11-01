function imglist = bb_getImages(image_directory)
    dinfo = dir('~/Campus_II/frames_alameda_noon_1_6_2017');
    imglist = {dinfo.name};
    imglist = natsortfiles(imglist);
    imglist(:,1:2) = [];
    imglist = strcat('~/Campus_II/frames_alameda_noon_1_6_2017', '/', imglist)
