# Multiple-camera tracking and pedestrian detection with BQP 

Working on Ubuntu 16.04, CUDA 8.0 and any GPU with >2GB of memory.

## Steps to install/Dependencies:

1. Assuming Ubuntu 16.04 (all the following steps might work on Ubuntu 14.04), `sudo apt-get update`.
2. Download + Install [Matlab 2016/2017](https://www.mathworks.com/downloads/)
3. (Optional) Download + Install [Computer Vision Toolbox v8.0](https://www.mathworks.com/products/computer-vision.html)
4. Download + Install [Caffe](https://github.com/BVLC/caffe)
5. Download + Install [RCNN](https://github.com/rbgirshick/rcnn)
6. Download + Install [DeepPed](https://github.com/DenisTome/DeepPed) in the rcnn directory, for all previous steps see [here](QUICKSTART.md)
7. Download + Install [IlogCPLEX](https://ibm.onthehub.com/WebStore/OfferingDetails.aspx?o=9b4eadea-9776-e611-9421-b8ca3a5db7a1) in a directory of your choosing, do not forget to change that directory in `settings/setTrackerParams.m`
8. Download + Install [CVX](http://cvxr.com/cvx/doc/install.html)
9. `cd && git clone https://github.com/pedro-abreu/mct-bqp`
10. `cd rcnn && sed -i '$ a addpath(genpath('~/mct-bqp'));' startup.m`
11. Your directory structure should now be *~/rcnn* in one folder and *~/mct-bqp* on another. You can install Caffe wherever you want.
12. Run the code from the rcnn folder, run either `campus2_script` or `hda_elevator_script`
13. Some utility functions from the [Dollar toolbox](https://github.com/pdollar/toolbox) are very useful for handling some of the data formats. This also has the ACF detector used for the HDA+ dataset.
14. You may need to have `g++` if you wish to use some `.mex` files for faster computation.
15. (Optional) There are some functions adapted from [vgg](http://www.robots.ox.ac.uk/~vgg/hzbook/code/) and from [Peter Kovesi](http://www.peterkovesi.com/matlabfns/) for RANSAC homography computation (found in `/homography/ransac`), if you wish to inspect them further.
16. (Optional) [Python 2.7.13](https://www.python.org/downloads/release/python-2713/)

Once you have done all the above and if you have installed rcnn in the `~/rcnn` directory (or wherever you have installed it), run `echo "addpath(genpath('~/mct-bqp'));" >> ~/rcnn/startup.m`.

Run `campus2_script.m` for tracking in the *Campus II* dataset (please run it from the `~/rcnn` or equivalent folder, without forgetting to run `startup.m`).

Run `hda_elevator_script.m` for tracking in a specific *HDA+* scenario (elevator lobby). You can run it from `/mct-bqp`.

*NOTE*: All data provided in the `hda_data` and `campus2_data` folders are not the actual datasets for privacy reasons but just txt results of the detections, which are useful to test the trackers. For the actual datasets see below.

### Tweaking/Finetuning

* To tweak the detection on the *HDA+* data, see `mct/settings/setDetectionParams_hda_elevator`. The comments should be enough for you to understand.

* To tweak detections on the *Campus_II* data, see `mct/settings/setDetectionParams_campus2`.

* To tweak the parameters of the actual tracker (which is equal for both datasets), see `mct/settings/setTrackerParams`.


### Datasets/files used and quick start guide

* To download the *Campus II dataset*, contact (acfbarata88@gmail.com). The dataset should be in a folder called `Campus_II` in your home directory.

* To download the *HDA+ dataset*, contact (alex@isr.ist.utl.pt). The dataset should be in a folder called `HDA_Dataset_V1.3` in your home directory.

* Any questions, contact me (pedro.f.abreu@ist.utl.pt)

* Several .mp4, .pdf and .png which are used in the code cannot be included since they are derived from the datasets. Others, like .txt's, are included.

* Including functions in scripts requires *MATLAB R2016b or later*.
~~~~~~~~~~~~~~~~
~ QUICKSTART.md provides a quick instalation guide for Caffe, RCNN and DeepPed, and how to set it up with our framework.
For the sake of completeness a short instalation guide to setup RCNN and DeepPed for Debian based distros is included.
~~~~~~~~~~~~~~~~
