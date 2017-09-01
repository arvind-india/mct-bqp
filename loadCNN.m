function [SVM detector rcnn_model cl2 PersonW PersonB] = loadCNN(LDCF_cascThr,LDCF_cascCal,LDCF_rescale)
    % load and adjust the LDCF detector
    % See toolbox/detector/acfModify and acfDetect for more info
    load('toolbox/detector/models/LdcfCaltechDetector.mat');
    pModify = struct('cascThr',LDCF_cascThr,'cascCal',LDCF_cascCal,'rescale',LDCF_rescale); %stride not included yet
    detector = acfModify(detector,pModify);

    % load the trained SVM
    SVM = load('data/rcnn_models/DeepPed/SVM_finetuned_alexnet.mat');
    PersonW = SVM.W; %Feature weights for scoring
    PersonB = SVM.b; %Constant scoring factor

    % load the trained svm of level 2
    cl2 = load('data/rcnn_models/DeepPed/SVM_level2.mat');

    %load the finetuned AlexNet
    use_GPU = 0;
    rcnn_model = rcnn_load_model('data/rcnn_models/DeepPed/finetuned_alexNet.mat', use_GPU); %0 is CPU, 1 is GPU, the file is the model file
