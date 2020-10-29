function [param_vec, target_struct, batch_img_pts, homographies] = estimate_params(img_arr, w_sq)
    % [Inputs]
    %   @img_arr: cell array contain img_arr
    %
    %   @w_sq: size of the checkerboard squares
    %
    % [Outputs]
    %   @param_vec: vector containing intrinsic & extrinsic parameters with
    %   the format
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
    %   @target_struct: geometry of the test target including
    %       - rows: number of rows
    %       - cols: number of columns
    %       - sq_size: size of the checkerboard squares
    %       - w_coord: world coordinates of the checkerboard points; shape N x 2
    %   @batch_img_pts: all detected points in images, shape N x 2 x
    %   img_num where N is the total number of calibration points in one image
    %   @homographies: cell array containing homographies
    %
    % [References]
    %   [1] Z. Chen, H. Liao and X. Zhang, "Telecentric stereo micro-vision
    %   system: Calibration method and experiments", Opt. Laser. Eng. 57, pp
    %   82-89 (2014)
    %
    %% Prepare for homography
    % Get board geometry
    [imagePoints,boardSize] = detectCheckerboardPoints(img_arr{1});
    rows = boardSize(1)-1;
    cols = boardSize(2)-1;
    num_pts = size(imagePoints,1);
    % Generate world coordinates
    vx = w_sq * (0:cols-1);
    vy = w_sq * (0:rows-1);
    [X, Y] = meshgrid(vx, vy);
    w_coord = [X(:) Y(:)];
    % Output structure
    target_struct.rows = rows;
    target_struct.cols = cols;
    target_struct.sq_size = w_sq;
    target_struct.w_coord = w_coord;
    % Set initial guess for u0, v0
    pxl_center = size(img_arr{1})/2;
    v0 = pxl_center(1);
    u0 = pxl_center(2);
    %% Solve for homographies & intrinsic matrices
    img_num = length(img_arr);
    homographies = cell(size(img_arr));
    K_mats = cell(size(homographies)); % Intrinsic estimations for each image
    batch_img_pts = zeros(num_pts, 2, img_num);
    for k=1:img_num
        % Find checkerboard points
        [imagePoints,~] = detectCheckerboardPoints(img_arr{k});
        batch_img_pts(:,:,k) = imagePoints; % grid points in one image
        vImgPts = imagePoints(:); % image points reshaped into 1D vector
        M = zeros(2*num_pts,6); % Least squares coefficient matrix
        % w_coord always identical - ranging from (0,0) ~ (74.8,19.8)
        M(1:num_pts,1:3) = [w_coord ones(num_pts,1)]; 
        M(num_pts+1:2*num_pts,4:6) = [w_coord ones(num_pts,1)];
        % Find homography (mapping from w_coord to imagePoints)
        h = M\vImgPts;
        homographies{k} = [reshape(h,3,[])';0 0 1];
        % magnification (different in practice from image to image)
        c1 = h(1)^2+h(2)^2+h(4)^2+h(5)^2;
        c2 = (h(1)*h(5)-h(2)*h(4))^2;
        p = [1 0 -c1 0 c2];
        ms = roots(p);
        m = max(ms);
        % Intrinsic matrix
        K_mats{k} = [m 0 u0; 0 m v0; 0 0 1];
    end
    %% Solve for global intrinsic matrix in least-sqaures sense
    param_vec = zeros(1, 10+5*img_num);
    % Ref: Chen 2014
    G = zeros(img_num, 4);
    G(:,1) = ones(img_num, 1);
    w = zeros(img_num, 1);
    for k = 1:img_num
        H = homographies{k};
        G(k,2) = -H(2,1)^2 - H(2,2)^2;
        G(k,3) = - H(1,1)^2 - H(1,2)^2;
        G(k,4) = 2 * (H(1,1)*H(2,1) + H(1,2)*H(2,2));
        w(k) = - (H(1,1) * H(2,2) - H(1,2) * H(2,1)) ^2;
    end
    l = G \ w;
    alpha = sqrt((l(2) * l(3) - l(4) ^ 2) / l(3));
    beta = sqrt(l(3));
    % gamma = sqrt(l(2) - l(1)/l(3)); % Skew is negligible
    K_lsq = [alpha 0 u0; 0 beta v0; 0 0 1];
    % Initial intrinsic parameters
    param_vec(1:10) = [alpha beta 0 u0 v0 0 0 0 0 0]; 
    %% Solve for extrinsics
    for k=1:img_num
        H = homographies{k};
        K = K_mats{k};
        E = K\H;  % Better to use different intrinsics to estimate extrinsics
    %     E = K_lsq \ H;
        % Translation
        tx = (H(1,3)-u0)/m;
        ty = (H(2,3)-v0)/m;
        % Find rotation matrix
        r13 = sqrt(1-E(1,1)^2-E(2,1)^2);
    %     r13 = r13 * isreal(r13); % Not needed if using different intrinsics
        r23 = sqrt(1-E(1,2)^2-E(2,2)^2);
    %     r23 = r23 * isreal(r23);
        r1 = [E(1:2,1); r13];
        r2 = [E(1:2,2); r23];
        r3 = cross(r1,r2);
        R_temp = [r1 r2 r3];
        [U,~,V] = svd(R_temp);
        R = U*V';
        v_rot = rotationMatrixToVector(R'); % MATLAB uses row vector definition
        param_vec(6+k*5:10+k*5) = [v_rot tx ty];
    end
end