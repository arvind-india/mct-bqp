function new_plane = draw_elevator_patio(gplane)
    axis([1.75 7.75 3 9]);
    % http://www.peteryu.ca/tutorials/matlab/plot_over_image_background
    new_plane = imagesc([1.75 7.75], [3 9], flipud(gplane));
    set(gca,'ydir','normal');
