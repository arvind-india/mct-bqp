# TODO Load data from groundplane detections
    gnd_detections = cell(length(cameras), 1)

    for id = 1:
        length(cameras)
        filename_g = ['/home/pedro/mct-bqp/' dataset '_data/detections-gndplane/' num2str(cameras{id}) '.txt']
        temp_g = csvread(filename_g)
        for i = 1:
            size(temp_g, 1)
            gnd_detections{id}{i} = temp_g(i, :)
        end
    end
    for id = 1:
        length(cameras)
        gnd_detections{id} = cell2mat(gnd_detections{id}')
        gnd_detections{id} = (accumarray(gnd_detections{id}(: , 2), (1: size(gnd_detections{id}, 1)).', [], @(x){gnd_detections{id}(x, : )}, {}))
    end


# TODO Load the images(needed for the appearance cues) and the ,mat files with the regions
cameraListImages = cell(length(cameras), 1)
inplanes = cell(length(cameras), 1)
for i = 1:
    length(cameras)
    cameraListImages{i} = loadImages(cameras, image_directories{i}, durations(i), 0, 'hda')
    inplanes{i} = load(strcat(visibility_regions_directory, num2str(cameras{i}), '.mat'))
    inplanes{i} = inplanes{i}.t
end
# TODO Load the homographies
[homographies, invhomographies] = loadHomographies(homography_directory, 'hda', cameras)
# TODO Use the homographies to compute the ground plane regions
ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'hda')
# TODO Compute overlap of regions
[overlap, ~, ~] = computeOverlap(ground_plane_regions)
