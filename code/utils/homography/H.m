function t = H(matrix, point)
    t = matrix * [point; 1];
    t(1) = t(1)/t(3);
    t(2) = t(2)/t(3);
    t = t(1:2);
end
