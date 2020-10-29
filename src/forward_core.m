function pts_i = forward_core(K, E, v_distort, pts_w, distort_model, distort_plane)
    % Description: core forward projection mode for a single image with
    % distortion.    
    % Inputs:
    %   K - Intrinsic matrix
    %   E - Extrinsic matrix
    %   v_distort - distortion coefficient vector
    %   pts_w - world coordinates of points
    %   distort_model - 'none', 'rad_tan' or 'full' (default)
    %   distort_plane - 'normal' (defualt) or 'pixel'
    % Outputs:
    %   pts_i - image coordinates of points. N x 2
    
    % Parsing inputs
    assert(ischar(distort_model));
    assert(ischar(distort_plane));
    % Forward projection
    N = size(pts_w,1);
    switch distort_plane
        case 'normal'
            % Compute distortion free point coordinates
            pts_w_temp = [pts_w ones(N,1)];
            pts_pre = pts_w_temp * E;
            % Apply distortion on normalized image coordiantes
            if strcmp(distort_model, 'none')
                pts_distort = pts_pre;
            else
                pts_distort = distort_v1(pts_pre, v_distort, distort_model, distort_plane);
            end
            if size(pts_distort,2)==2
               pts_distort = [pts_distort ones(size(pts_distort(:,1)))];
            end
            % Transform to pixel coordinate
            pts_temp = pts_distort* K;
            pts_i = pts_temp(:,1:2);
        case 'pixel'
            % Compute distortion free point coordinates
            pts_w_temp = [pts_w ones(N,1)];
            pts_pre = pts_w_temp * E * K;
            % Apply distortion
            if strcmp(distort_model, 'none')
                pts_i = pts_pre(:,1:2);
            else
                pts_i = distort_v1(pts_pre, v_distort, distort_model, distort_plane);
            end
        otherwise
            error(['Unrecognized distortion plane ' distort_plane]);
    end            
end