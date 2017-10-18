function [out, bin_img] = convertBB2bin_img(bb, bb_img, ref_img)
    ref_subImage = imcrop(ref_img, bb(3:6));
    test_subImage = imcrop(bb_img, bb(3:6));
    %Convert RGB 2 HSV Color conversion
    background_hsv = round(rgb2hsv(ref_subImage));
    bb_hsv = round(rgb2hsv(test_subImage));

    out = imsubtract(bb_hsv,background_hsv);
    %out = bitxor(bb_hsv,background_hsv);
    %Convert RGB 2 GRAY
    %out = rgb2gray(out);

    %Read Rows and Columns of the Image
    [rows columns rgb] = size(out);
    bin_img = zeros(rows, columns);
    %Convert to Binary Image
    for i=1:rows
        for j=1:columns
            if (abs(out(i,j,1)) + abs(out(i,j,2)) + abs(out(i,j,3))) > 1
                bin_img(i,j) = 1;
            else
                bin_img(i,j) = 0;
            end
        end
    end
