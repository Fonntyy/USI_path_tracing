function outputImage = crossBilateralFilter(inputImage, depthMap, normalMap, domainSigma, depthSigma, normalSigma)
% aided by depth map and normal map.
%
% parameters:
%      1. inputImage - double RGB image [H x W x 3]
%      2. depthMap - float depth (Inf = background) [H x W]
%      3. normalMap - unit normals (zero vector = background) [H x W x 3]
%      4. domainSigma - domain Gaussian filter sigma (pixels, e.g. 3)
%      5. depthSigma - normalized depth range filter sigma (fraction of [0,1], e.g. 0.1)
%      6. normalSigma - normal range filter sigma (radians, e.g. 0.3)

    bgMask = isinf(depthMap);

    finiteDepths = depthMap(~bgMask);
    minDepth = min(finiteDepths);
    maxDepth = max(finiteDepths);
    normalizedDepthMap = (depthMap - minDepth) / (maxDepth - minDepth);
    normalizedDepthMap(bgMask) = Inf;

    halfKernelSize = round(2 * domainSigma);
    [X, Y] = meshgrid(-halfKernelSize:halfKernelSize, -halfKernelSize:halfKernelSize);
    domainKernel = exp(-(X.^2 + Y.^2) / (2 * domainSigma^2));

    paddedImage = padarray(inputImage, [halfKernelSize halfKernelSize], 'replicate');
    % added pixels will not influence image anyway since we consider them as background
    paddedDepth = padarray(normalizedDepthMap, [halfKernelSize halfKernelSize], Inf);
    paddedNormal = padarray(normalMap, [halfKernelSize halfKernelSize 0], 0);
    paddedBgMask = padarray(bgMask, [halfKernelSize halfKernelSize], true);

    [rows, cols, ~] = size(inputImage);
    outputImage = zeros(rows, cols, 3);

    for r = 1:rows
        for c = 1:cols

            % do not filter background pixels as we cannot compare depths
            % and normals of neighboring pixels, simply copy
            if bgMask(r, c)
                outputImage(r, c, :) = inputImage(r, c, :);
                continue
            end

            rShifted = r + halfKernelSize;
            cShifted = c + halfKernelSize;

            imgWin = paddedImage(rShifted-halfKernelSize:rShifted+halfKernelSize, cShifted-halfKernelSize:cShifted+halfKernelSize, :);
            depthWin = paddedDepth(rShifted-halfKernelSize:rShifted+halfKernelSize, cShifted-halfKernelSize:cShifted+halfKernelSize);
            normalWin = paddedNormal(rShifted-halfKernelSize:rShifted+halfKernelSize, cShifted-halfKernelSize:cShifted+halfKernelSize, :);
            bgWinMask = paddedBgMask(rShifted-halfKernelSize:rShifted+halfKernelSize, cShifted-halfKernelSize:cShifted+halfKernelSize);

            depthDifference = depthWin - normalizedDepthMap(r, c);
            depthWeight = exp(-(depthDifference.^2) / (2 * depthSigma^2));
            depthWeight(bgWinMask) = 0;

            n = reshape(normalMap(r, c, :), 1, 1, 3);
            dotProduct = sum(normalWin .* n, 3); % dot product = cos(angle b/w normals) since normals are normalized
            angle = acos(dotProduct);
            normalWeight = exp(-(angle.^2) / (2 * normalSigma^2));
            normalWeight(bgWinMask) = 0;

            W = domainKernel .* depthWeight .* normalWeight;
            Wsum = sum(W(:));

            for ch = 1:3
                outputImage(r, c, ch) = sum(W .* imgWin(:,:,ch), "all") / Wsum;
            end
        end
        if mod(r, 50) == 0
            fprintf('%d / %d\n', r, rows);
        end
    end

    fprintf('Done.\n');
end


function depthMap = loadRawDepthMap(filePath, width, height)
% Headerless float32 little-endian depth file -> [H x W] double.
% Inf background

    fid = fopen(filePath, 'rb', 'ieee-le');
    if fid == -1, error('Cannot open: %s', filePath); end
    [raw, count] = fread(fid, width*height, 'single');
    fclose(fid);

    if count ~= width*height
        error('Size mismatch: expected %d floats (%d bytes), got %d.\nCheck width=%d height=%d.', ...
              width*height, width*height*4, count, width, height);
    end

    depthMap = double(reshape(raw, width, height)');
end


function normalMap = loadRawNormalMap(filePath, width, height)
% Headerless float32 little-endian normal map -> [H x W x 3] double.
%
% Assumes normals are stored going row by row throught the images as: X Y Z X Y Z ... 
%
% (0,0,0) background

    fid = fopen(filePath, 'rb', 'ieee-le');
    if fid == -1, error('Cannot open: %s', filePath); end
    [raw, count] = fread(fid, width*height*3, 'single');
    fclose(fid);

    if count ~= width*height*3
        error('Size mismatch: expected %d floats, got %d.\nCheck width/height.', ...
              width*height*3, count);
    end

    % raw is [width*height*3 x 1] as XYZXYZXYZ...
    % Reshape to [3 x width x height] then permute to [height x width x 3]
    raw        = reshape(raw, 3, width, height);   % [3 x W x H]
    normalMap  = double(permute(raw, [3 2 1]));    % [H x W x 3]
end


% ======================== MAIN ================================

inputImage = im2double(imread('result.ppm'));
[height, width, ~] = size(inputImage);

depthMap = loadRawDepthMap ('result_depth.raw',  width, height);
normalMap = loadRawNormalMap('result_normal.raw', width, height);

domainSigma = 6;
depthSigma = 0.3;
normalSigma = 0.3;

filtered = crossBilateralFilter(inputImage, depthMap, normalMap, domainSigma, depthSigma, normalSigma);

imwrite(filtered, 'filtered_result.png');

