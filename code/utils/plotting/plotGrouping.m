function plotGrouping(T,G)
    figure;
    ax1 = subplot(2,1,1);
    title(ax1,'Matrix T');
    imagesc(T);
    colorbar;
    ax2 = subplot(2,1,2);
    imagesc(G);
    colorbar;
    title(ax2,'Matrix G, (Laplacian matrix)');
end
