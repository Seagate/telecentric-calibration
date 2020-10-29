function pts_i = batch_forward_model_v1(param_vec, pts_w, varargin)
    % @description forward projection model given calibration coordinates
    % and array of intrinsic/extrinsic parameters
    %
    % @param param_vec: vector containing camera model parameters, length =
    % 10 + 5 * n where n is number of calibration images. The vector is
    % specified as:
    %   param_vec = [alpha, beta, gamma, u0, v0, k1, p1, p2, q1, q2, ra1, rb1, rc1, tx1,
    %   ty1, ..., ran, rbn, rcn, txn, tyn]
    %   where the parameters are defined as follows:
    %       Intrinsics:
    %           alpha, beta - lens scaling factor
    %           gamma - lens skew factor
    %           [u0, v0] - lens distortion center 
    %           [k1, p1, p2, q1, q2] - lens distortion coefficients as defined in
    %           ref [2]. k1 is for radial distortion; p1, p2 are for
    %           tangential distortion; thin prism distortion s1, s2 can be
    %           found from q1, s2 as s = q - p.
    %       Extrinsics:
    %           [ra, rb, rc] - 3D rotation vector
    %           [tx, ty] - 2D translation vector
    % @param pts_w: calibration points in world coordinates dimension:
    %   n x 2; n is number of points in each image.
    %
    % @optional distortion: distortion model to use, can be {'none',
    % 'full', 'rad_tan'}
    %
    % @return pts_i: calibration points in image coordinates, size: n x 2 x
    % num_images.
    
    % Parse input options
    p = inputParser;
    addParameter(p, 'distort_model', 'full', @ischar);
    addParameter(p, 'distort_plane', 'normal', @ischar);
    addParameter(p, 'isotropic', false, @islogical);
    addParameter(p, 'skew', false, @islogical);
    parse(p, varargin{:});
    distort_model = p.Results.distort_model;
    distort_plane = p.Results.distort_plane;
    % Numerical counts
    N = size(pts_w, 1);
    ex_param_num = length(param_vec)-10;
    assert(~mod(ex_param_num,5),"Number of parameters must be multiples of 5");
    num_images = ex_param_num/5;
    pts_i = zeros(N, 2,num_images);
    % Extract parameters
    alpha = param_vec(1);
    beta = param_vec(2);
    gamma = param_vec(3);
    u0 = param_vec(4);
    v0 = param_vec(5);
    v_distort = param_vec(4:10);
    % Construct intrinsic matrix
    K = [alpha 0 0; 0 beta 0; u0 v0 1];
    if p.Results.skew
        K(2,1) = gamma; % introduces dependency on gamma
    end
    if p.Results.isotropic
        K(2,2) = alpha; % eliminates dependency on beta
    end
    % Loop over images
    for k=1:num_images
        vRot = param_vec(5*k+6:5*k+8);
        vT = param_vec(5*k+9:5*k+10);
        % Convert rotation matrix
        R = rotationVectorToMatrix(vRot); % MATLAB defines rotation matrix with row vectors
        % Construct extrinsic matrix
        E = [R(1:2,1:2) [0;0]; vT 1];
        % Core forward projection for one image
        pts_i(:,:,k) = forward_core(K, E, v_distort, pts_w, distort_model, distort_plane);
    end
end