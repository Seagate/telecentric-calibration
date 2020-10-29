function imgs_correct = correctDistortion(images, v_intrinsic, varargin)
    assert(iscell(images));
    assert(isvector(v_intrinsic));
    assert(isnumeric(v_intrinsic));
    assert(length(v_intrinsic)==10);
    % Parse input options
    p = inputParser;
    addParameter(p, 'distort_model', 'full', @ischar);
    addParameter(p, 'distort_plane', 'normal', @ischar);
    parse(p, varargin{:});
    distort_model = p.Results.distort_model;
    distort_plane = p.Results.distort_plane;
    % Correct
    imgs_correct = cell(size(images));
    distort_coeff = v_intrinsic(4:10);
    switch distort_plane
        case 'pixel'
            inversefn = @(pts) distort_v1(pts, distort_coeff, distort_model, 'pixel');
            tform = geometricTransform2d(inversefn);
            for k=1:length(images)
                imgs_correct{k} = imwarp(images{k},tform);    
            end
        case 'normal'
            K = [v_intrinsic(1) 0 0; v_intrinsic(3) v_intrinsic(2) 0;...
                v_intrinsic(4) v_intrinsic(5) 1];
            inversefn = @(pts) apply_norm_distort(pts, K, distort_coeff, distort_model);
            tform = geometricTransform2d(inversefn);
            for k=1:length(images)
                imgs_correct{k} = imwarp(images{k},tform);    
            end
        otherwise
               error("Unknown distortion plane")
    end            
end

%% Utility function
function out = apply_norm_distort(pts_i, K, coeff, model)
    pts_normal = [pts_i ones(size(pts_i,1),1)] / K;
    pts_distort = distort_v1(pts_normal, coeff, model, 'normal');
    pts_distort_pxl = [pts_distort ones(size(pts_distort(:,1)))] * K;
    out = pts_distort_pxl(:,1:2);
end