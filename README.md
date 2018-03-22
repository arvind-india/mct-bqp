# Multiple-camera tracking with BQP

Working on Ubuntu 16.04, 16GB of RAM, CUDA 8.0 and GTX 1080 Ti 11 Gb (these last two are for detection)

## Steps to install/Dependencies:

1. Download + Install [Matlab 2016/2017](https://www.mathworks.com/downloads/)
2. (Optional) Download + Install [Computer Vision Toolbox v8.0](https://www.mathworks.com/products/computer-vision.html)
3. Download + Install [IlogCPLEX](https://ibm.onthehub.com/WebStore/OfferingDetails.aspx?o=9b4eadea-9776-e611-9421-b8ca3a5db7a1) in a directory of your choosing, do not forget to change that directory in `setTrackerParams.m`. Academic licenses are available.
4. Download + Install [CVX](http://cvxr.com/cvx/doc/install.html)
5. `cd && git clone https://github.com/pedro-abreu/mct-bqp`
6. Run `startup.m`.
7. The scripts for tracking are all in the `mct-bqp/mct` folder.
8. Some utility functions from the [Dollar toolbox](https://github.com/pdollar/toolbox) are very useful for handling some of the data formats. This also has the ACF detector used for the HDA+ dataset.
9. You may need to have `g++` if you wish to use some `.mex` files for faster computation.
10. (Optional) There are some functions adapted from [vgg](http://www.robots.ox.ac.uk/~vgg/hzbook/code/) and from [Peter Kovesi](http://www.peterkovesi.com/matlabfns/) for RANSAC homography computation (found in `/homography/ransac`), if you wish to inspect them further.
11. (Optional) [Python 2.7.13](https://www.python.org/downloads/release/python-2713/)

<!-- Add some 2-cam images of the datasets used -->

Run `tracking_campus.m` for tracking in the *Campus II* dataset. Run `tracking_hda.m` for tracking in a specific *HDA+* scenario (hall sequence).

*NOTE*: All data provided in the `hda_data` and `campus2_data` folders are not the actual datasets for privacy reasons but just .txt results of the detections, which are useful to test the trackers. For the actual datasets see below. Also note that there is a bug in MATLAB for the recent versions of Ubuntu if you're trying to convert avi/mp4 files to image sequences. To use our included script `video2frames.m` you have to launch MATLAB as `LD_PRELOAD=/usr/lib64/libstdc++.so.6 /usr/local/bin/matlab -desktop` so it knows to use the library from your files ystem and not the packaged one.

### Tweaking/Finetuning

* To tweak the parameters of the tracker see `/mct/setTrackerParams` or each of the tracking files. It is also possible to tweak the detectors.

### Datasets/files used and quick start guide

* To download the *Campus II dataset*, contact (acfbarata88@gmail.com). The dataset should be in a folder called `Campus_II` in your home directory.

* To download the *HDA+ dataset*, contact (alex@isr.ist.utl.pt). The dataset should be in a folder called `HDA_Dataset_V1.3` in your home directory. We include .txt's of some specific test sequences and the parsed ground truths, for convenience sake.

* Both the EPFL and UCLA (which they call CAMPUS, a name which we avoid so as not to generate confusion) datasets are publicly available (and labeled, with their respective homographies) at https://bitbucket.org/merayxu/multiview-object-tracking-dataset. The datasets can be either on your home folder or in the `/ucla_data` and `/epfl_data` folders in this repository.

* Any questions, contact me (pedro.f.abreu@ist.utl.pt) or post an issue. Suggestions are accepted.

### Pedestrian Detector

If you wish to use DeepPed pedestrian detector you need to go through the following steps:

1. Download + Install [Caffe](https://github.com/BVLC/caffe)
2. Download + Install [RCNN](https://github.com/rbgirshick/rcnn)
3. Download + Install [DeepPed](https://github.com/DenisTome/DeepPed) in the rcnn directory, for all previous steps see [here](QUICKSTART.md)

~~~~~~~~~~~~~~~~
~ DETECTOR_QUICKSTART.md provides a quick installation guide for Caffe, RCNN and DeepPed for the sake of completeness and how to set it up with our framework.
~~~~~~~~~~~~~~~~
