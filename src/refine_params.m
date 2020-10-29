function fit_struct = refine_params(v_param0, target_struct, batch_img_pts, varargin)
    % Description: refine camera model prameters based on linear least
    % squares estimation
    %
    % Inputs:
    %   @v_param0: initial guess for camera parameters
    %   @target_struct: struct containing test target geometry
    %   @batch_img_pts: detected checkerboard points in all images
    %   @varargin: other options for the forward model
    % 
    % Output:
    %   @fit_struct: fitting result containing intrinsic and extrinsic
    %   paramters
    %
    % Optimization
    f = @(param, pts) batch_forward_model_v1(param, pts, varargin{:});
    history_full = runlsqcurvefit(f, v_param0, target_struct.w_coord, batch_img_pts);
    % param_fit_img_plane = history_full.x(end,:);
    fit_struct = struct;
    fit_struct.vector = history_full.x(end,:);
    % Intrinsic vector (K elements + distortion)
    fit_struct.intrinsics = fit_struct.vector(1:10);
    v = fit_struct.intrinsics;
    % Calibration matrix
    fit_struct.K = [v(1) v(3) v(4); 0 v(2) v(5); 0 0 1];
    % Distortion coefficients
    fit_struct.distort_coeff = v(6:10);
    % Extrinsics
    fit_struct.extrinsics = reshape(fit_struct.vector(11:end),5,[]);    
end