# Multiple-camera tracking and pedestrian detection with BQP - v0.5

## Steps to install/Prerequisites:

1. Download + Install [Matlab 2016/2017](https://www.mathworks.com/downloads/)
2. (Optional) Download + Install [Computer Vision Toolbox v8.0](https://www.mathworks.com/products/computer-vision.html)
3. Download + Install [Caffe](https://github.com/BVLC/caffe)
4. Download + Install [RCNN](https://github.com/rbgirshick/rcnn)
5. Download + Install [DeepPed](https://github.com/DenisTome/DeepPed) in the rcnn directory, for all previous steps see [here](QUICKSTART.md)
6. Download + Install [IlogCPLEX](https://ibm.onthehub.com/WebStore/OfferingDetails.aspx?o=9b4eadea-9776-e611-9421-b8ca3a5db7a1) in a directory of your choosing, do not forget to change that directory in `settings/setTrackerParams.m`
7. Download + Install [CVX](http://cvxr.com/cvx/doc/install.html)
8. `cd rcnn/DeepPed/`
9. `git clone https://github.com/pedro-abreu/campus2_code`
10. `cd rcnn`
11. `sed -i '$ a addpath(genpath('DeepPed/campus2_code'));' startup.m`
12. Your directory structure should now be *rcnn/DeepPed/campus2_code*. You can install Caffe wherever you want.
13. Run the code from the rcnn folder, run either `campus2_script` or `hda_elevator_script`
14. Some utility functions from the [Dollar toolbox](https://github.com/pdollar/toolbox) are very useful for handling some of the data formats. This also has the ACF detector used for the HDA+ dataset.
15. You may need to have `g++` if you wish to use some `.mex` files for faster computation.
16. (Optional) There are some functions adapted from [vgg](http://www.robots.ox.ac.uk/~vgg/hzbook/code/) and from [Peter Kovesi](http://www.peterkovesi.com/matlabfns/) for RANSAC homography computation (found in /homography/ransac), if you wish to inspect them further.
17. (Optional) [Python 2.7.13](https://www.python.org/downloads/release/python-2713/)

Once you have done all the above and if you have installed rcnn in the `~/rcnn` directory, run `echo "addpath(genpath('~/mct-bqp'));" >> ~/rcnn/startup.m` and that should be it. Furthermore if you wish to run `campus2_script.m` and run the CNN detections please run it from the `rcnn` folder.

*NOTE*: All data provided in the hda_data and campus2_data are not the actual datasets for privacy reasons but just text results of the detections, which are useful to test the trackers. For the actual datasets see below.

### Datasets/files used and quick start guide

* To download the *Campus II dataset*, contact (acfbarata88@gmail.com). The dataset should be in a folder called `Campus_II` in your home directory.

* To download the *HDA+ dataset*, contact (alex@isr.ist.utl.pt). The dataset should be in a folder called `HDA_Dataset_V1.3` in your home directory.

* Any questions, contact me (pedro.f.abreu@ist.utl.pt)

* Several .mp4, .pdf and .png which are used in the code cannot be included since they are derived from the datasets. Others, like .txt's, are included.

* Including functions in scripts requires *MATLAB R2016b or later*.
~~~~~~~~~~~~~~~~
~ [Quick instalation guide][QUICKSTART.md] for Caffe, RCNN and DeepPed, and how to set it up with our framework
For the sake of completeness a short instalation guide to setup RCNN and DeepPed for Debian based distros is included.
~~~~~~~~~~~~~~~~
