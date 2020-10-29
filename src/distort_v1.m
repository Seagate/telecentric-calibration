% Description: lens distortion model
% References:
% [1] J. Wang, F. Shi, J. Zhang and Y. Liu, "A new calibration model of
% camera lens distortion", Pattern Recognit., 41 pp.607-615 (2008)

function pts_distort = distort_v1(pts_pre, v_param, model, distort_plane)
    % Inputs:
    %   @pts_pre - coordinates of undistorted points, size Nx2
    %   @v_param - distortion related parameters[u0, v0, k1, p1, p2, q1, q2]
    %   @model - full distortion model or raidial + tangential only
    %   @distort_plane - apply distortion on normal or sensor plane
    %
    % Outputs:
    %   @pts_distort - coordinates of distorted points, size Nx2
    %
    u0 = v_param(1);
    v0 = v_param(2);
    k1 = v_param(3);
    p1 = v_param(4);
    p2 = v_param(5);
    q1 = v_param(6);
    q2 = v_param(7);
    % Apply distortion
    switch distort_plane
        case 'pixel'
            u_s = pts_pre(:,1) - u0;
            v_s = pts_pre(:,2) - v0;
        case 'normal'
            u_s = pts_pre(:,1);
            v_s = pts_pre(:,2);
        otherwise
                error("unrecognized distortion plane");
    end            
    r = sqrt(u_s.^2+v_s.^2);
    switch model
        case 'full' % radial + tangential + thin prism
            du = k1 * u_s .* r.^2 + p1 * u_s.^2 + p2 * u_s .* v_s + q1 * r.^2;
            dv = k1 * v_s .* r.^2 + p2 * v_s.^2 + p1 * u_s .* v_s + q2 * r.^2;
        case 'rad_tan' % No benefit by including higherr order radial terms
            du = k1 * u_s .* r.^2 + p1 * (2 * u_s.^2 + r.^2) + 2 * p2 * u_s .* v_s;
            dv = k1 * v_s .* r.^2 + p2 * (2 * v_s.^2 + r.^2) + 2 * p1 * u_s .* v_s;
    end
    pts_distort = [pts_pre(:,1) + du, pts_pre(:,2) + dv];    
end