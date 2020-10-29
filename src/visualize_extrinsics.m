function visualize_extrinsics(extrinsic_stack, target_struct, varargin)    
    % Parse input options
    p = inputParser;
    addParameter(p, 'z_offset', 100, @isnumeric);
    addParameter(p, 'origin', 'camera', @ischar);
    parse(p, varargin{:});
    z_offset = p.Results.z_offset;    
    coord_origin = p.Results.origin;
    % Target geometry
    width = target_struct.sq_size * (target_struct.cols - 1);
    height = target_struct.sq_size * (target_struct.rows - 1);
    target_shape = [0 0 0; 0 height 0; width height 0; width 0 0];
    % Get rotation/translation of each target
%     extrinsic_stack = reshape(param_vec(11:end),5,[]);
    % Plot
    num_imgs = size(extrinsic_stack,2);
    switch coord_origin
        case 'camera'
            for k=1:num_imgs
                [R_mat, Txy] = extractRTxy(extrinsic_stack(:,k));
                T = [Txy; z_offset-R_mat(1,3)*width/2];
                target_cam = target_shape * R_mat + T';
                target_cam_plot = [target_cam; target_cam(1,:)];
                plot3(target_cam_plot(:,1), target_cam_plot(:,2), target_cam_plot(:,3));
                hold on;
            end
            plotCamera;
        case 'target'
            for k = 1:num_imgs
                [R_mat, Txy] = extractRTxy(extrinsic_stack(:,k));
                T = -R_mat * [Txy; z_offset];
                plotCamera('Location', T, 'Orientation', R_mat);               
                hold on;
            end
            target_plot = [target_shape; target_shape(1,:)];
            plot3(target_plot(:,1), target_plot(:,2), target_plot(:,3));
        otherwise
                error("unknown origin type");
    end
    grid on;
    daspect([1,1,1]);
    xlabel('x');
    ylabel('y');
    zlabel('z');
    hold off;
end

%% Utility
function [R, Txy] = extractRTxy(param_vec)
    R_vec = param_vec(1:3);
    R = rotationVectorToMatrix(R_vec);
    Txy = param_vec(4:5);
end