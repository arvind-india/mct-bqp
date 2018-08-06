function plotAppearanceValues(i,n,k,c_a,cands_homo_percam,cameras)
    chunks = reshape(c_a,k,n);
    figure; hold on;
    colormap(hot);
    title(['Displaying values of each ground plane candidate (Not representative of the actual samples) @ ' num2str(cameras{i})]);
    for t = 1:size(cands_homo_percam{i},2)
        for j = 1:k
            sz = 300;
            scatter(cands_homo_percam{i}{t}(j,1), cands_homo_percam{i}{t}(j,2),sz,chunks(j,t),'square','filled');
        end
    end
    colorbar;
end
