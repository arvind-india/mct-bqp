# Multiple-camera tracking with BQP - v0.11

## Steps to install/Prerequisites:

1. Download + Install [Matlab 2016/2017](https://www.mathworks.com/downloads/)
2. Download + Install [Caffe](https://github.com/BVLC/caffe)
3. Download + Install [RCNN](https://github.com/rbgirshick/rcnn)
4. Download + Install [DeepPed](https://github.com/DenisTome/DeepPed) in the rcnn directory
5. Download + Install [IlogCPLEX](https://ibm.onthehub.com/WebStore/OfferingDetails.aspx?o=9b4eadea-9776-e611-9421-b8ca3a5db7a1) in a directory of your choosing, do not forget to change that directory in `settings/setTrackerParams.m`
6. `cd rcnn/DeepPed/`
7. `git clone https://github.com/pedro-abreu/campus2_code`
8. `cd rcnn`
9. `sed -i '$ a addpath(genpath('DeepPed/campus2_code'));' startup.m`
10. Your directory structure should now be *rcnn/DeepPed/campus2_code*. You can install Caffe wherever you want.
11. Run the code from the rcnn folder, run either `campus2_script` or `hda_elevator_script`
12. (Optional) Some utility functions from the [Dollar toolbox](https://github.com/pdollar/toolbox) are very useful for handling some of the data formats.

*NOTE*: All data provided in the hda_data and campus2_data are not the actual datasets for privacy reasons but actual text results of the detections, which are useful to test the trackers. For the actual datasets see below.

### Datasets used and quick start guide

* To download the *campus_2 dataset*, contact (acfbarata88@gmail.com)

* To download the *HDA+ dataset*, contact (alex@isr.ist.utl.pt)

* Any questions, contact me (pedro.f.abreu95@ist.utl.pt)

~~~~~~~~~~~~~~~~
~ Quick instalation guide for RCNN and DeepPed
For the sake of completeness a short instalation guide to setup RCNN and DeepPed for Ubuntu based distros is included below:
~~~~~~~~~~~~~~~~

1. boost, protobuf, hdf5, glog, gflags, general dependencies:
```
	sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
	sudo apt-get install --no-install-recommends libboost-all-dev
	sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
```
2. CUDA 7+: Get your file from the NVidia website, I recommend the .deb for Ubuntu (some rumours said it was broken but I haven't had that experience), the file name should be something similar to
`cuda-repo-ubuntu1604-8-0-local-ga2_8.0.61-1_amd64.deb`. More instructions can be found at the CUDA NVidia website

3. ATLAS/BLAS:
```
	sudo apt-get install libatlas-base-dev
	sudo apt-get install libopenblas-dev
```
4. (Optional Python):
```
	sudo apt install python-dev
```
* In `caffe-0.999-master` take a look and edit `Makefile.config` details. Be specially careful where you point caffe to.

* Then follow the steps (if you get key=-2 at the end it means everything was installed correctly)
```
	cd CAMPUS_II_PEDESTRIAN_TRACKING/software
	cd caffe-0.999-master
	sudo make clean
	sudo make all
	sudo make test
	sudo make matcaffe
	cd data/ilsvrc12
	sudo ./get_ilsvrc_aux.sh
	cd ../../../rcnn
	ln -sf ~/hda_code/CAMPUS_II_PEDESTRIAN_TRACKING/software/caffe-0.999-master external/caffe
	sudo ./selective_search/fetch_selective_search.sh
	sudo ./data/fetch_models.sh
	sudo ./data/fetch_selective_search_data.sh
	sudo matlab
	>>rcnn_build()
	>>key = caffe('get_init_key')
```
* Firstly try to run the demo on the CPU mode and PASCAL data (the training may take a while, be patient):
```
	>> rcnn_demo('PASCAL',0)
```
* If all goes well try the GPU version (The CPU version should work properly, however the GPU version may have several issues. One I got was a MATLAB crash followed by: `Check failed: error == cudaSuccess (8 vs. 0)  invalid device function *** Check failure stack trace: *** Killed`. To solve this add the next two lines to the Makefile.config in its section (# CUDA architecture setting: going with all of them, you may also remove the following architectures since those are to be deprecated: `CUDA_ARCH := -gencode arch=compute_20,code=sm_20 \ -gencode arch=compute_20,code=sm_21 \`). For my GPU (GTX960M I used the following):
```
		-gencode arch=compute_50,code=sm_50 \
		-gencode arch=compute_50,code=compute_50
```
and run `make clean`, `make all` again in the caffe-master directory. Another you may get is `Check failed: error == cudaSuccess (2 vs. 0) out of memory`. You may want to change the batchsize to a lower value (default is at 20).

* After this, with all GPU issues solved, run:
```
	>> rcnn_demo
```
* If both demos work correctly, still on the rcnn directory, move on to DeepPed (if you want git clone https://github.com/DenisTome/DeepPed.git and git clone https://github.com/pdollar/toolbox.git) and try it with CPU mode:
```
	./DeepPed/fetch_models.sh
	sudo nano startup.m
	(nano - add at the end) addpath(genpath('DeepPed')); and addpath(genpath('toolbox'));
	cd DeepPed
	sudo nano deepPed_demo
	(nano - edit) use_gpu=0
	>>deepPed_demo
```
WARNING: This should already be resolved, but if you see the following and MATLAB segfaults with `[libprotobuf ERROR google/protobuf/text_format.cc:274] Error parsing text-format caffe.NetParameter: 7:7: Message type "caffe.NetParameter" has no field named "layer". WARNING: Logging before InitGoogleLogging() is written to STDERR F0630 15:03:01.788671  6076 upgrade_proto.cpp:571] Check failed: ReadProtoFromTextFile(param_file, param) Failed to parse NetParameter file: model-defs/alexnet_deploy_fc7_CAFFE.prototxt` replace the `rcnn/model-defs/alexnet_deploy_fc7_CAFFE.prototxt` with the one in `DeepPed/mode_def`.

* To test the gpu CUDA support change to `use_gpu=1` and run `deepPed_demo` again. Make sure your GPU has enough memory! If it does not you may get an error like `Check failed: error == cudaSuccess (2 vs 0)`. To fix this you have to reduce the batchsize. To do this you'll have to find `rcnn/model-defs/alexnet_deploy_fc7_CAFFE.prototxt` in the original neural network and change the input dimensions to half of what they are (keep the one that is 3 unchanged). This implies the finetuned SVM in the code has to be changed as well in `CNNdetect.m`, and you'll end up with half the number of features but something that can run.
