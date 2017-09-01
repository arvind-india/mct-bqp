%NOTE: Make these 2 dimensional because we may want the Neural Net to get different capture params for the two cameras
global LDCF_cascThr
LDCF_cascThr = -1;

global LDCF_cascCal
LDCF_cascCal = 0.028;

global LDCF_rescale
LDCF_rescale = 1.0;

global pNMS_overlap
pNMS_overlap = 0.9;

global LDCF_stride
LDCF_stride = 4;

global score_threshold
score_threshold = [0.5 0.9];

global NMS_maxoverlap
NMS_maxoverlap = [0.9 0.9];
