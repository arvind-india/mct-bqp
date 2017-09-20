function [homographies, invhomographies] = loadHomographies_campus2(filename)
    homographies_struct = load(filename);
    homographies{1} = homographies_struct.TFORM_alameda.tdata.T;
    homographies{2} = homographies_struct.TFORM_central.tdata.T;
    invhomographies{1} = homographies_struct.TFORM_alameda.tdata.Tinv;
    invhomographies{2} = homographies_struct.TFORM_central.tdata.Tinv;
