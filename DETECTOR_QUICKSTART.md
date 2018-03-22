## Quickstart RCNN and DeepPed

1. boost, protobuf, hdf5, glog, gflags, general dependencies:
```
	sudo apt install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
	sudo apt install --no-install-recommends libboost-all-dev
	sudo apt install libgflags-dev libgoogle-glog-dev liblmdb-dev
```
2. CUDA 7+: Get your file from the NVidia website, I recommend the .deb for Ubuntu (some rumours said it was broken but I haven't had that experience), the file name should be something similar to
`cuda-repo-ubuntu1604-8-0-local-ga2_8.0.61-1_amd64.deb`. More instructions can be found at the CUDA NVidia website. Alternatively you may try to do:
```
	sudo apt install nvidia-cuda-toolkit
	nvcc --version
```
This will succeed but from my experience won't give you good enough results. Follow this http://www.pradeepadiga.me/blog/2017/03/22/installing-cuda-toolkit-8-0-on-ubuntu-16-04/ to get a working solution. You may install cuda-9.0. If you don't install with `apt-get` then you may have to add an export PATH to your bashrc.

3. ATLAS/BLAS:
```
	sudo apt install libatlas-base-dev
	sudo apt install libopenblas-dev
```
If you opt to build from source you may have something that will report throtling issues and require you to install fortran, for that simply do:
```
	sudo apt install gfortran
```
4. (Optional Python):
```
	sudo apt install python-dev
```
* In `caffe-0.999` take a look and edit `Makefile.config` details (rename the example to `Makefile.config`). Be specially careful where you point caffe to (matlab directory, should be in `usr/local/MATLAB`).

* Before you start be careful to point also to the correct place where you previously installed CUDA. Caffe 0.99 has a couple of bugs we have to fix first: hdf5, old CUDA architectures and signbit. First, execute this script https://github.com/BVLC/caffe/issues/2347. Then:
```
cd /usr/lib/x86_64-linux-gnu
sudo ln -s libhdf5_serial.so.8.0.2 libhdf5.so
sudo ln -s libhdf5_serial_hl.so.8.0.2 libhdf5_hl.so
```
If this symlink doesn't work try:
```
sudo ln -s /usr/lib/x86_64-linux-gnu/libhdf5_serial.so.10 /usr/lib/x86_64-linux-gnu/libhdf5.so
sudo ln -s /usr/lib/x86_64-linux-gnu/libhdf5_serial_hl.so.10 /usr/lib/x86_64-linux-gnu/libhdf5_hl.so
```
Then remove both lines that start with `-gencode arch=compute_20`.
Then edit `caffe-0.999/include/caffe/util/math_functions.hpp` and apply the following changes. From:
```
using std::signbit;
DEFINE_CAFFE_CPU_UNARY_FUNC(sgnbit, y[i] = signbit(x[i]));
```
to:
```
// using std::signbit;
DEFINE_CAFFE_CPU_UNARY_FUNC(sgnbit, y[i] = std::signbit(x[i]));
```
When trying to compile by using `sudo make matcaffe` Makefile change `CXXLIBS="$$CXXLIBS $(LDFLAGS)" -o $@` to `CXXLIBS="$$CXXLIBS $(STATIC_NAME) $(LDFLAGS)" -output $@`. Even if you get warnings, if you get `MEX compiled successfully` it worked.

* Then follow the steps (if you get key=-2 at the end it means everything was installed correctly)
```
	cd caffe-0.999
	sudo make clean
	sudo make all
	sudo make matcaffe
	sudo make time
	sudo make test
	cd data/ilsvrc12
	sudo ./get_ilsvrc_aux.sh
	cd ../../../rcnn
	ln -sf ~/caffe-0.999 external/caffe
	sudo ./selective_search/fetch_selective_search.sh
	sudo ./data/fetch_models.sh
	sudo ./data/fetch_selective_search_data.sh
	sudo matlab
	>> rcnn_build()
	>> key = caffe('get_init_key')
```
If you're running it with MATLAB2017 you may have to do ```export LD_PRELOAD=$LD_PRELOAD:/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libprotobuf.so.9``` before. Put it in your `~/.bashrc` or `~/.xinitrc` file.

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
	>> deepPed_demo
```
WARNING: This should already be resolved, but if you see the following and MATLAB segfaults with `[libprotobuf ERROR google/protobuf/text_format.cc:274] Error parsing text-format caffe.NetParameter: 7:7: Message type "caffe.NetParameter" has no field named "layer". WARNING: Logging before InitGoogleLogging() is written to STDERR F0630 15:03:01.788671  6076 upgrade_proto.cpp:571] Check failed: ReadProtoFromTextFile(param_file, param) Failed to parse NetParameter file: model-defs/alexnet_deploy_fc7_CAFFE.prototxt` replace the `rcnn/model-defs/alexnet_deploy_fc7_CAFFE.prototxt` with the one in `DeepPed/mode_def`.

* To test the gpu CUDA support change to `use_gpu=1` and run `deepPed_demo` again. Make sure your GPU has enough memory! If it does not you may get an error like `Check failed: error == cudaSuccess (2 vs 0)`. To fix this you have to reduce the batchsize. To do this you'll have to find `rcnn/model-defs/alexnet_deploy_fc7_CAFFE.prototxt` in the original neural network and change the input dimensions to half of what they are (keep the one that is 3 unchanged). This implies the finetuned SVM in the code has to be changed as well in `CNNdetect.m`, and you'll end up with half the number of features but something that can run.
