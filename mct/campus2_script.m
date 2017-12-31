addpath('/home/pedro/rcnn');
% Set params of the detection
setDetectionParams_campus2;
% Set params of the tracker
setTrackerParams;
% Pick if you want to display debug images or not
show_images = 'on';
set(0,'DefaultFigureVisible',show_images);

%%=========================================================
% Load images
cameraListImages = cell(2,1);
if strcmp(order, 'toolbox')
  % NOTE: this was in a wrong order, we had to do our own cust bbGt function
  for i=1:length(cameras)
      cameraListImages{i} = bbGt('getFiles',image_directories(i));
  end
elseif strcmp(order, 'cardinal')
  for i=1:length(cameras)
      cameraListImages{i} = loadImages(cameras, image_directories{i}, 0, i, 'campus2');
  end
end

% Use the trained Neural Network to do pedestrian detection (transfer learning)
%sample_size = 300;
sample_size = 12;
allDetections = CNNdetect(cameraListImages, sample_size);

% allDetections are all detections on each frame


%%=========================================================
inplanes{1} = [1 436; 1022 409; 1022 766; 0 766]; % Alameda cam, these points were given to us
inplanes{2} = [3 391; 328 391; 384 295; 475 295; 988 550; 930 622; 1 688]; % Central cam, these points were given to us
% Parse for systematic error detections and remove all empty cells
allDetections = filterCNN(allDetections, inplanes);
for i=1:2
    allDetections{i} = allDetections{i}(~cellfun('isempty',allDetections{i}));
end
% 3D plot over frames
identifyPotentialStaticObjects(allDetections);
%%=======================================================
% Plot images to show detections (with bounding boxes)
figure; show_detections = 'slideshow';
if strcmp('slideshow', show_detections) == 1
    start_frame = 1;
    for start_frame = start_frame:4:sample_size
        waitforbuttonpress;
        clf;
        plotDebugBoundingBoxes(cameraListImages,allDetections,start_frame,'campus_2');
    end
elseif strcmp('4frames', show_detections) == 1
    start_frame = 1;
    plotDebugBoundingBoxes(cameraListImages,allDetections,start_frame,'campus_2');
end
%%=======================================================
% Plot detections of all peds in both cameras (we consider that peds are represented by the middle of their BB's)
plotDetectionsCameraSpace(cameraListImages,allDetections,'campus_2');
%%=======================================================
% Load homographies, these homographies where obtained via the use of cp2tform
[homographies, invhomographies] = loadHomographies(homography_directory,'campus_2'); % Defined in global variables
% Plot regions
%inplanes{1} = [494 545; 426 687; 602 681; 852 590; 631 539]; % Alameda cam
%inplanes{2} = [162 510; 702 608; 917 558; 603 390; 447 412]; % Central cam
%inplanes{1} = [1 436; 1022 409; 1022 766; 0 766]; % Alameda cam
%inplanes{2} = [3 391; 328 391; 384 295; 475 295; 988 550; 930 622; 1 688]; % Central cam
ground_plane_regions = computeGroundPlaneRegions(inplanes, homographies, length(cameras), 'campus_2');

%%==============================================================
colors = {'Red','Green'};
for i=1:length(cameras)
    drawPoly(ground_plane_regions{i},colors{i},0.5,false);
end
% Plot pedestrians
plotDetectionsGroundPlane(allDetections,homographies,ground_plane_regions,'show_outliers','campus_2');
% Plot overlap of camera regions
[overlap, ~, ~] = computeOverlap(ground_plane_regions);
drawPoly(overlap,'Black',1.0,false);
