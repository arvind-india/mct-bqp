function drawGrid(gplane, grid_granularity, dataset)
    if strcmp('campus_2', dataset)
        [X,Y] = meshgrid(min(gplane(1,:)):grid_granularity:max(gplane(1,:)), min(gplane(2,:)):grid_granularity:max(gplane(2,:)));
        mesh(X,Y,X*0,'edgecolor', rgb('Beige'));
    end
    if strcmp('hda',dataset)
        M = size(gplane,1); N = size(gplane,2);
        grid_granularity = 10;
        for k = 1:grid_granularity:M
            x = [1 N]; y = [k k];
            plot(x,y,'Color',rgb('Beige'),'LineStyle','-'); plot(x,y,'Color',rgb('Beige'),'LineStyle',':');
        end
        for k = 1:grid_granularity:N
            x = [k k]; y = [1 M];
            plot(x,y,'Color',rgb('Beige'),'LineStyle','-'); plot(x,y,'Color',rgb('Beige'),'LineStyle',':');
        end
    end
