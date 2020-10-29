function [calib_struct, target_struct] = calibrate(images, sq_w, varargin)
    % Description: wrapper function for parameter estimation and refinement
    %
    % Inputs
    %   @images - cell array containing image arrays
    %   @sq_w - size of the checkerboard squares
    %   @varargin - additional arguments for forward model in refine_params()
    %
    % Outputs
    %   @calib_struct - struct containing calibration results
    %   @target_struct - struct containing geometry of test target
    %
    % Estimation
    [v_param0, target_struct, batch_img_pts, ~] = estimate_params(images, sq_w);
    % Refinment
    calib_struct = refine_params(v_param0, target_struct, batch_img_pts, varargin{:});    
end