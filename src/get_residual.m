function res = get_residual(w_coord, batch_img_pts, type, arg1, varargin)
    % ** Inputs ** 
    %   @w_coord: N x 2 array containing world coordinates of calibration
    %   points. N is number of calibration points per image.
    %   @batch_img_pts: N x 2 x m array containing detected calibration points in
    %   the images.
    %   @type: 'homographic' or 'parametric'
    %   @arg1: homographies or parameter vector depending on the type
    %   variable. The homographies should be a 1-D cell array. The
    %   parameter vector has a length of 10 + 5 * m where m is the number
    %   of images.
    %   @[distort_plane]: 'pixel' or 'normal' (default)
    %   @[distort_model]: 'full' (default), 'rad_tan' or 'none'
    %
    % ** Output **
    %   @res: defined as the 2-norm of the residual vector
       
    % Calculate residual
    switch type
        case 'homographic'
            homographies = arg1;
            assert(iscell(homographies), "homographies must be cell array!");
            batch_proj_pts = zeros(size(batch_img_pts));
            num_pts = size(batch_img_pts,1);
            for k = 1:length(homographies)
                 temp = [w_coord ones(num_pts,1)]* homographies{k}';   
                 batch_proj_pts(:,:,k) = temp(:,1:2);
            end
            M_homo_err = batch_proj_pts - batch_img_pts;
            res = norm(M_homo_err(:));
        case 'parametric'
            param_vec = arg1;
            assert(isnumeric(param_vec), "param_vec must be numerical array");
            assert(isvector(param_vec), "param_vec must be 1-D vector!");
            assert(~mod(length(param_vec)-10,5), "param_vec must have length 10+5*m");
            pts_i_nl = batch_forward_model_v1(param_vec, w_coord, varargin{:});
            M_err_nl = pts_i_nl - batch_img_pts;
            res = norm(M_err_nl(:));
        otherwise
                error("Unknown projection type!");
    end          
end