# campus2_code

## Steps to install::

1. Download + Install [RCNN](https://github.com/rbgirshick/rcnn)
2. Download + Install [DeepPed](https://github.com/DenisTome/DeepPed) in the rcnn directory
3. git clone this repository in rcnn/DeepPed/
4. cd rcnn
5. ```sed -i '$ a addpath(genpath('DeepPed/campus2_code'));' startup.m```
6. ```sed -i '$ a path(path,genpath('DeepPed/campus2_code/utils'));' startup.m```
7. ```sed -i '$ a path(path,genpath('DeepPed/campus2_code/constraints'));' startup.m```
8. Run the code from the rcnn folder


