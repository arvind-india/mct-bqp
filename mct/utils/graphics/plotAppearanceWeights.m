function plotAppearanceWeights(weights)
    % plotting of the weights
    for i = 1:2
        for j = 1:size(weights{i},2)
            subplot(3,3,(i-1)*3+j);
            col = weights{i}(:,j);
            bar(col, 'k');
            hold on
            plot([256 256],[-0.5 0.5],'r');
            plot([512 512],[-0.5 0.5],'g');
            plot([768-10 768-10],[-0.5 0.5],'b');
        end
    end
end
