function x = reshapeX(x, cameras, k, targs_percam)
    % Separate x into cells for each camera
    x = mat2cell(x, [size(targs_percam{1},1)*k*2 size(targs_percam{2},1)*k*2]);
    % Separate targets from clone targets
    for c = 1:length(cameras)
        x{c} = mat2cell(x{c}, [length(x{c})/2 length(x{c})/2]);
        % Split array into array with n columns, 1:2 because of clones
        for cl = 1:2
            x{c}{cl} = reshape(x{c}{cl}, k,length(x{c}{cl})/k,1);
        end
    end
end
