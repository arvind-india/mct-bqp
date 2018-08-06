function normalized_array = normalize(array,k,N)
    array = cell2mat(array);
    % Normalize for each candidate
    array = reshape(array,k,N);
    for t = 1:size(array,2)
        array(:,t) = array(:,t)./abs(min(array(:,t)));
    end
    normalized_array = reshape(array,k*N,1);
end
