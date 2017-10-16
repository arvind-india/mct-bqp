# Multiple-camera tracking with BQP - v0.11

## Steps to install/Prerequisites:

1. Download + Install [Matlab 2016/2017](https://www.mathworks.com/downloads/)
2. Download + Install [Caffe](https://github.com/BVLC/caffe)
3. Download + Install [RCNN](https://github.com/rbgirshick/rcnn)
4. Download + Install [DeepPed](https://github.com/DenisTome/DeepPed) in the rcnn directory, for all previous steps see [here[«][QUICKSTART.md]
5. Download + Install [IlogCPLEX](https://ibm.onthehub.com/WebStore/OfferingDetails.aspx?o=9b4eadea-9776-e611-9421-b8ca3a5db7a1) in a directory of your choosing, do not forget to change that directory in `settings/setTrackerParams.m`
6. `cd rcnn/DeepPed/`
7. `git clone https://github.com/pedro-abreu/campus2_code`
8. `cd rcnn`
9. `sed -i '$ a addpath(genpath('DeepPed/campus2_code'));' startup.m`
10. Your directory structure should now be *rcnn/DeepPed/campus2_code*. You can install Caffe wherever you want.
11. Run the code from the rcnn folder, run either `campus2_script` or `hda_elevator_script`
12. (Optional) Some utility functions from the [Dollar toolbox](https://github.com/pdollar/toolbox) are very useful for handling some of the data formats.
13. You may need to have `g++` if you wish to use some `.mex` files for faster computation.

*NOTE*: All data provided in the hda_data and campus2_data are not the actual datasets for privacy reasons but actual text results of the detections, which are useful to test the trackers. For the actual datasets see below.

### Datasets used and quick start guide

* To download the *campus_2 dataset*, contact (acfbarata88@gmail.com)

* To download the *HDA+ dataset*, contact (alex@isr.ist.utl.pt)

* Any questions, contact me (pedro.f.abreu@ist.utl.pt)

~~~~~~~~~~~~~~~~
~ Quick instalation guide for RCNN and DeepPed
For the sake of completeness a short instalation guide to setup RCNN and DeepPed
for Ubuntu based distros is included.
~~~~~~~~~~~~~~~~
