function scores = helperHammingDistance(features1, features2)
%helperHammingDistance compute hamming distance between two groups of
%   binary feature vectors.
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.

%   Copyright 2019 The MathWorks, Inc.

persistent lookupTable; % lookup table for counting bits

N1 = size(features1, 1);
N2 = size(features2, 1);

scores = zeros(N1, N2);

if isempty(lookupTable)
    lookupTable = zeros(256, 1);
    for i = 0:255
        lookupTable(i+1) = sum(dec2bin(i)-'0');
    end
end

for r = 1:N1
    for c = 1:N2
        temp = bitxor(features1(r, :), features2(c, :));
        idx = double(temp) + 1; % cast needed to avoid integer math
        scores(r,c) = sum(lookupTable(idx));
    end
end

end