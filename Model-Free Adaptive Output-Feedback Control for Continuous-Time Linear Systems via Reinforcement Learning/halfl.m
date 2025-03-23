function vec = halfl(matrix)
    [m, n] = size(matrix);
    vec = zeros((m + 1) * n  / 2, 1);
    index = 1;
    for i = 1:m
        for j = i:n
            if i~=j
                vec(index) = 2*matrix(i, j);
            else
                vec(index) = matrix(i, j);
            end
            index = index + 1;
        end
    end
end