# UCLA CAMPUS dataset
## There are several remarks to be made:
1. There are 4 subsets
2. We focused mostly on the Parkinglot which has 4 cams: view-GL1, view-GL2, view-GL5, view-GL6
3. We have extracted the images (@30fps) for the 4 cameras. The resolution of the extracted images is not 1920x1080, but 1080x576 (see below).
4- If you wish to use the homographies (K, C, R matrices are provided for each camera) and if you wish to use the provided FOV regions/masks, then you need to use resized images AND resized detections (as all the detections are in the 1920x1080 res).
5. The original files are slightly parsed already to be comma separated and have only numeric data
6. For the parsed results we keep interpolated data from the original authors (merayxu)
