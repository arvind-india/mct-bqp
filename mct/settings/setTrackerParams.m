global hdaRootDirectory
hdaRootDirectory ='/home/pedro/HDA_Dataset_V1.3';

global cplex_folder
cplex_folder = '/opt/ibm/ILOG/CPLEX_Studio1271/cplex/matlab/x86-64_linux';
addpath(cplex_folder);

global FW_max_iterations
FW_max_iterations = 5000;  % max number of iteration, I found out the opts.TOL (tolerance?) is more important in the convergence in my problem

global FW_duality_gap
FW_duality_gap = 2; % This sets the duality gap and you can change it if you want to change the tolerance for FW convergence

global FW_eps
FW_eps = 1e-12; % (eps in matlab = 1e-16) not sure why this is needed


global g_candidates
g_candidates = 5; % basis for the number of candidates

global lambda
lambda = 0.1; % variable for the appearance cues

global zeta
zeta = 0.3; % weight of the motion constraint

global eta
eta = 0.2; % weight of the neighbourhood motion constraint

global score_threshold
score_threshold = 0.5;

global d256_metric
d256_metric = 'chisq';

global d1_metric
d1_metric = 'euclidean';

global gating_distance
gating_distance = 6.0; % This does actually correspond to 6 meters

global delta
delta = 10;
