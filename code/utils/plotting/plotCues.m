function plotCues(a, m, b, dendrograms, next_images, cands_percam, k, num_cams, comfort_distance)
    figure;
    count = 1;
    for c = 1:num_cams
        ac = a{c};
        % Cut clones off
        ac = ac(1:length(ac)/2,:);
        n = length(ac)/k;
        chunks = reshape(ac,k,n,1);
        subplot(4,2,count);
        count = count + 1;
        h = imshow(next_images{c});
        hold on;
        colormap(gca, hot);
        set(h, 'AlphaData', 0.5)
        for t = 1:size(cands_percam{c},2)
            sz = 300;
            scatter(cands_percam{c}{t}(:,1) + cands_percam{c}{t}(:,3)/2, cands_percam{c}{t}(:,2) + cands_percam{c}{t}(:,4)/2, sz, chunks(:,t), 'square','filled');
        end
        colorbar;
        title(['pdf app cam ' num2str(c) ', t+1 (squares are placed on BB centroid)']);
    end
    for c = 1:num_cams
        mc = m{c};
        mc = mc(1:length(mc)/2,:);
        n = length(mc)/k;
        chunks = reshape(mc,k,n,1);
        subplot(4,2,count);
        count = count + 1;
        h = imshow(next_images{c});
        hold on;
        colormap(gca, summer);
        set(h, 'AlphaData', 0.5)
        for t = 1:size(cands_percam{c},2)
            sz = 300;
            scatter(cands_percam{c}{t}(:,1) + cands_percam{c}{t}(:,3)/2, cands_percam{c}{t}(:,2) + cands_percam{c}{t}(:,4)/2, sz, chunks(:,t), 'square','filled');
        end
        colorbar;
        title(['pdf motion cam ' num2str(c) ', t+1 (squares are placed on BB centroid)']);
    end
    for c = 1:num_cams
        bc = b{c};
        bc = bc(1:length(bc)/2,:);
        n = length(bc)/k;
        chunks = reshape(bc,k,n,1);
        subplot(4,2,count);
        count = count + 1;
        h = imshow(next_images{c});
        hold on;
        colormap(gca, pink);
        set(h, 'AlphaData', 0.5)
        for t = 1:size(cands_percam{c},2)
            sz = 300;
            scatter(cands_percam{c}{t}(:,1) + cands_percam{c}{t}(:,3)/2, cands_percam{c}{t}(:,2) + cands_percam{c}{t}(:,4)/2, sz, chunks(:,t), 'square','filled');
        end
        colorbar;
        title(['pdf bounds cam ' num2str(c) ', t+1 (squares are placed on BB centroid)']);
    end
    for c = 1:num_cams
        subplot(4,2,count);
        count = count + 1;
        if isempty(dendrograms{c}) == 0
            dendrogram(dendrograms{c},'ColorThreshold',1.5)
            title(['Cluster dendrogram cam ' num2str(c) ', max in-cluster dist.' num2str(max(dendrograms{c}(:,3))) ' m, cutoff dist: ' num2str(comfort_distance)])
        end
    end
end
