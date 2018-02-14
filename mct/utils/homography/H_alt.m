function t = H_alt(matrix, point)
    t = [point 1] * matrix;
    t(1) = t(1)/t(3);
    t(2) = t(2)/t(3);
    t = t(1:2);
end
