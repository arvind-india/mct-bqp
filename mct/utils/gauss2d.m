function mat = gauss2d(gsize, sigma, center)
    % Generic 2d gaussian function without cross-correlations (circular gaussian)
    % Inputs
    %  gsize: gridsize for the gaussian evaluation
    %  sigma: correlation matrix
    %  center: mean

    % Output
    %  mat: matrix of the distribution
    [R,C] = ndgrid(1:1:gsize(1), 1:1:gsize(2));
    mat = gaussC(R,C,sigma,center);

function val = gaussC(x,y,sigma,center)
    xc = center(1);
    yc = center(2);
    exponent = ((x-xc).^2)/(2*sigma(1,1).^2)+((y-yc).^2)/(2*sigma(2,2).^2);
    val = 1*exp(-exponent);
