# Multi-camera tracker with Binary Quadratic Programming

Formulates tracking as a *BQP (Binary Quadratic Programming)* problem, solved with *Frank-Wolfe* optimization. Uses *KCF* filters, *motion vector* gaussians and clustering (*K-Means*) for group dynamics (quadratic cues).
Working on Ubuntu 16.04, 16GB of RAM, CUDA 8.0 and GTX 1080 Ti 11 Gb.

### Prerequisites:

1. Download + Install [Matlab 2016/2017](https://www.mathworks.com/downloads/)
2. Download + Install Parallel Computing Toolbox (for GPU acceleration).
3. (Optional) Download + Install [Computer Vision Toolbox v8.0](https://www.mathworks.com/products/computer-vision.html)
4. Download + Install [IlogCPLEX](https://ibm.onthehub.com/WebStore/OfferingDetails.aspx?o=9b4eadea-9776-e611-9421-b8ca3a5db7a1) in a directory of your choosing, do not forget to change that directory in `setTrackerParams.m`. Academic licenses are available.
5. Download + Install [CVX](http://cvxr.com/cvx/doc/install.html)
6. `cd && git clone https://github.com/pedro-abreu/mct-bqp`
7. [Dollar toolbox](https://github.com/pdollar/toolbox) are very useful for handling some of the data formats. This also has the ACF detector used for the HDA+ dataset.
8. `g++` for `.mex` file compilation
10. (Optional) There are some functions adapted from [vgg](http://www.robots.ox.ac.uk/~vgg/hzbook/code/) and from [Peter Kovesi](http://www.peterkovesi.com/matlabfns/) for RANSAC homography computation.
### Data

* All the used text files to test the trackers can be found [here](https://drive.google.com/drive/folders/1rMIsHtYdiJKqvP191rhEX01fbp5Akgu_)

* *Campus II* -- contact (acfbarata88@gmail.com). However, the original dataset is unlabeled. We provide a small sequence that is labeled, so we can evaluate the tracker.

* *HDA+ dataset* -- contact (alex@isr.ist.utl.pt). The dataset should be in a folder called `HDA_Dataset_V1.3` in your home directory. We include .txt's of some specific test sequences and the parsed ground truths, for convenience sake.

* *EPFL and UCLA* datasets are publicly available and labeled [here](https://bitbucket.org/merayxu/multiview-object-tracking-dataset).

* Any questions, contact me (pedro.f.abreu@ist.utl.pt) or post an issue.
