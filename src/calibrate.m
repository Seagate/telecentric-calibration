function [calib_struct, target_struct] = calibrate(images, sq_w, varargin)    
    % Estimation
    [v_param0, target_struct, batch_img_pts, ~] = estimate_params(images, sq_w);
    % Refinment
    calib_struct = refine_params(v_param0, target_struct, batch_img_pts, varargin{:});    
end